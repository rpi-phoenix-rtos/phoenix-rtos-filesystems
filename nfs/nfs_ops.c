/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2)
 *
 * mt* -> libnfs handlers. See nfs_ops.h for the contract. The mapping mirrors
 * dummyfs's handler semantics (return chars-consumed from lookup, one dirent
 * per readdir keyed by a cumulative cookie, otSymlink-aware read/unlink) while
 * swapping dummyfs's in-RAM object store for libnfs sync calls against the
 * remote export.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>   /* PATH_MAX (symlink readlink staging buffer) */
#include <poll.h>
#include <unistd.h>   /* usleep (transient-error retry backoff) */
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <phoenix/attribute.h>

#include <nfsc/libnfs.h>

#include "nfs_ops.h"


/* libnfs returns negative errno-ish on failure; clamp to a sane VFS errno. */
static int nfs_err(int rc)
{
	if (rc >= 0) {
		return rc;
	}
	int e = -rc;
	/* libnfs occasionally returns -1 with the detail in nfs_get_error(); map
	 * that to a generic I/O error rather than -EPERM. */
	if (e == 1) {
		return -EIO;
	}
	return -e;
}


static int nfs_typeFromMode(uint64_t mode)
{
	if (S_ISDIR(mode)) {
		return otDir;
	}
	if (S_ISLNK(mode)) {
		return otSymlink;
	}
	if (S_ISCHR(mode) || S_ISBLK(mode) || S_ISFIFO(mode)) {
		return otDev;
	}
	return otFile;
}


/* Is this a transient RPC error worth retrying (connection reset / timeout)? A genuine
 * missing entry (ENOENT) or other definite error is not. */
static int nfs_transient(int rc)
{
	int e = nfs_err(rc);
	return (e == -EIO) || (e == -ETIMEDOUT);
}


/* Refresh a node's cached type via a don't-follow stat. Returns 0 or -errno. Bounded retry on
 * transient RPC errors (contributes to the intermittent exec -5: the mtOpen path stats first). */
static int nfs_refreshStat(nfs_fs_t *fs, nfs_node_t *n, struct nfs_stat_64 *st)
{
	int rc = -EIO;
	for (int tries = 0; tries < 25; tries++) {
		rc = nfs_lstat64(fs->nfs, n->path, st);
		if ((rc == 0) || !nfs_transient(rc)) {
			break;
		}
		usleep(tries < 6 ? (10000u << tries) : 640000u);
	}
	if (rc != 0) {
		return nfs_err(rc);
	}
	n->type = nfs_typeFromMode(st->nfs_mode);
	return 0;
}


/* Bounded client-state reclaims per operation. An NFSv4 lease/state expiry is
 * recovered by rebuilding the client (nfs_reclaim); this caps how many times a
 * single op will do so before giving up, so a genuinely dead server still errors
 * out promptly rather than looping. */
#define NFS_RECLAIM_MAX 2


/* Does rc indicate the server has discarded our NFSv4 client state (idle lease
 * lapse, server restart, or accumulated stale state across rapid reboots)?
 *
 * libnfs's check_nfs4_error maps NFSv4 status codes through the NFSv3 errno
 * table (nfs_v4.c), which has no entry for the v4-only state codes, so they all
 * surface as -ERANGE with the real status preserved only in the error string.
 * We therefore match the status name in nfs_get_error() to tell a recoverable
 * expiry apart from a genuine -ERANGE (NFS3ERR_DQUOT). Only meaningful when rc is
 * an error and the string was set by this very call (all these ops call
 * nfs_set_error on failure), so gate on rc < 0. */
static int nfs_isStateExpiry(nfs_fs_t *fs, int rc)
{
	if (rc >= 0) {
		return 0;
	}
	const char *e = nfs_get_error(fs->nfs);
	if (e == NULL) {
		return 0;
	}
	return (strstr(e, "NFS4ERR_EXPIRED") != NULL) ||
		(strstr(e, "NFS4ERR_STALE_CLIENTID") != NULL) ||
		(strstr(e, "NFS4ERR_STALE_STATEID") != NULL) ||
		(strstr(e, "NFS4ERR_BAD_STATEID") != NULL);
}


/* Re-establish NFSv4 client state after an expiry. Rebuild the libnfs context
 * from scratch: nfs_mount re-runs SETCLIENTID + SETCLIENTID_CONFIRM (the RFC 7530
 * reclaim), using our stable client name so the server REPLACES the lapsed state
 * rather than accumulating a second incarnation. Every cached filehandle belonged
 * to the old context (freed by nfs_destroy_context) and its open stateid is dead,
 * so they are all invalidated; the id<->path table is untouched and each retried
 * op re-opens by path. Runs on the loop thread (no concurrent libnfs access).
 * Returns 0 on success, -errno on failure (the old context is kept on failure so
 * the caller can still surface the original error). */
static int nfs_reclaim(nfs_fs_t *fs)
{
	struct nfs_context *fresh = nfs_makeContext(fs->version);
	if (fresh == NULL) {
		return -ENOMEM;
	}

	if (nfs_mount(fresh, fs->server, fs->export) != 0) {
		printf("nfs-fs: reclaim re-mount %s:%s failed: %s\n", fs->server, fs->export, nfs_get_error(fresh));
		nfs_destroy_context(fresh);
		return -EIO;
	}

	struct nfs_context *old = fs->nfs;
	fs->nfs = fresh;
	nfs_node_invalidateHandles(&fs->nodes);
	nfs_destroy_context(old);

	printf("nfs-fs: reclaimed NFSv4 client state (re-mounted %s:%s)\n", fs->server, fs->export);
	return 0;
}


/* If rc is a recoverable NFSv4 state expiry and reclaim budget remains, rebuild
 * the client and return 1 (caller should retry the op); otherwise return 0. */
static int nfs_tryReclaim(nfs_fs_t *fs, int rc, int *budget)
{
	if ((*budget <= 0) || (nfs_isStateExpiry(fs, rc) == 0)) {
		return 0;
	}
	(*budget)--;
	return (nfs_reclaim(fs) == 0) ? 1 : 0;
}


int nfs_ops_renew(nfs_fs_t *fs)
{
	int rc = nfs_renew(fs->nfs);
	if (rc == 0) {
		return 0;
	}

	/* Renew failed. If the lease has already lapsed, reclaim now so the next
	 * open() doesn't have to; any other (e.g. transient) failure is left for the
	 * next tick / the per-op reclaim safety net. */
	if (nfs_isStateExpiry(fs, rc) != 0) {
		printf("nfs-fs: renew found lease expired, reclaiming\n");
		return nfs_reclaim(fs);
	}
	return rc;
}


int nfs_ops_lookup(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *res, oid_t *dev)
{
	nfs_node_t *d;
	int len = 0;

	if (dir == NULL) {
		d = nfs_node_find(&fs->nodes, NFS_ROOTID);
	}
	else if (dir->port != fs->port) {
		return -EINVAL;
	}
	else {
		d = nfs_node_find(&fs->nodes, dir->id);
	}
	if (d == NULL) {
		return -ENOENT;
	}

	/* Resolve component-by-component (mirrors dummyfs len bookkeeping). */
	const char *cur = d->path;
	nfs_node_t *node = d;

	while (name[len] != '\0') {
		while (name[len] == '/') {
			len++;
		}
		if (name[len] == '\0') {
			break;
		}

		/* ".." at the export root crosses back to the parent fs. */
		const char *end = name + len;
		while ((*end != '\0') && (*end != '/')) {
			end++;
		}
		size_t comp = (size_t)(end - (name + len));
		if ((comp == 2) && (strncmp(name + len, "..", 2) == 0) && (node->id == NFS_ROOTID)) {
			*res = fs->parent;
			*dev = fs->parent;
			return len + 2;
		}

		char nm[256];
		if (comp >= sizeof(nm)) {
			return -ENAMETOOLONG;
		}
		memcpy(nm, name + len, comp);
		nm[comp] = '\0';

		char *child = nfs_node_joinPath(cur, nm);
		if (child == NULL) {
			return -ENOMEM;
		}

		struct nfs_stat_64 st;
		/* Bounded retry on transient RPC errors (connection reset/timeout); a genuine missing
		 * entry (ENOENT) breaks immediately. Otherwise a transient stat failure during path
		 * resolution fails the whole open/exec (contributes to the intermittent exec -5). */
		int rc = -EIO;
		for (int tries = 0; tries < 25; tries++) {
			rc = nfs_lstat64(fs->nfs, child, &st);
			if (rc == 0) {
				break;
			}
			int e = nfs_err(rc);
			if (e != -EIO && e != -ETIMEDOUT) {
				break;
			}
			usleep(tries < 6 ? (10000u << tries) : 640000u);
		}
		if (rc != 0) {
			free(child);
			/* Route through nfs_err so a transient RPC error (EIO/ESTALE/
			 * ETIMEDOUT) is reported as itself rather than masked as "no such
			 * file". A genuine missing entry still maps to -ENOENT (libnfs
			 * returns -ENOENT for NFS*ERR_NOENT). */
			return nfs_err(rc);
		}

		nfs_node_t *cn = nfs_node_get(&fs->nodes, child);
		free(child);
		if (cn == NULL) {
			return -ENOMEM;
		}
		cn->type = nfs_typeFromMode(st.nfs_mode);

		len += comp;
		node = cn;
		cur = cn->path;

		/* Mountpoint: a child fs is spliced here (mtSetAttr(atDev)). Hand back
		 * the child oid and stop — the kernel re-resolves the remaining path in
		 * the child fs (mirrors dummyfs.c:104-109). This is what makes /dev
		 * reachable after the NFS export takes over "/" (#153 T3 design-A). */
		if (cn->mnt.port != 0) {
			*res = cn->mnt;
			*dev = cn->mnt;
			return len;
		}

		/* If the resolved component is not a directory but more path remains,
		 * fail like dummyfs. */
		if ((name[len] != '\0') && (cn->type != otDir)) {
			/* skip trailing slashes to see if anything meaningful remains */
			int p = len;
			while (name[p] == '/') {
				p++;
			}
			if (name[p] != '\0') {
				return -ENOTDIR;
			}
		}
	}

	res->port = fs->port;
	res->id = node->id;
	/* If the final node is itself a mountpoint, hand back the spliced child as
	 * the dev oid so the kernel redirects into it (e.g. a bare lookup of /dev
	 * after takeover — #153 T3 design-A). Otherwise dev == the node itself. */
	*dev = (node->mnt.port != 0) ? node->mnt : *res;

	return len;
}


int nfs_ops_open(nfs_fs_t *fs, oid_t *oid)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	/* Lazy-close fast path (#156): the node is parked on the idle LRU with its
	 * fh still open (refs went to 0 without nfs_close — see nfs_ops_close). The
	 * loader faulting the next page lands here. Reuse the fh, skip both the
	 * nfs_open and the nfs_refreshStat RPC. Skipping the re-stat is safe: lazy-
	 * close already gives up server-side-redeploy detection for a cached fh (the
	 * fh would point at the old inode regardless), so the stat buys nothing here
	 * — and a file the loader is actively paging in is not being redeployed. */
	if ((n->idle != 0) && (n->fh != NULL)) {
		nfs_node_idleUnlink(&fs->nodes, n);
		n->refs++;
		return 0;
	}

	/* Re-stat on open so a redeployed file isn't shadowed (OQ-B). */
	struct nfs_stat_64 st;
	int rc = nfs_refreshStat(fs, n, &st);
	if (rc != 0) {
		return rc;
	}

	/* Directories and symlinks are handled by path-based ops, not an fh. */
	if ((n->type == otFile) && (n->fh == NULL)) {
		int reclaimBudget = NFS_RECLAIM_MAX;
		struct nfsfh *fh = NULL;
		for (;;) {
			rc = nfs_open(fs->nfs, n->path, O_RDWR, &fh);
			if (rc != 0) {
				/* Fall back to read-only (e.g. mode lacks write), bounded-retrying transient RPC
				 * errors — this open is on the exec path, so a transient failure here is a prime
				 * cause of the intermittent exec -5. */
				for (int tries = 0; tries < 25; tries++) {
					rc = nfs_open(fs->nfs, n->path, O_RDONLY, &fh);
					if ((rc == 0) || !nfs_transient(rc)) {
						break;
					}
					usleep(tries < 6 ? (10000u << tries) : 640000u);
				}
			}
			if (rc == 0) {
				break;
			}
			/* NFSv4 client state expired (idle lease lapse / server state loss):
			 * re-establish it and retry the open. */
			if (nfs_tryReclaim(fs, rc, &reclaimBudget) != 0) {
				continue;
			}
			return nfs_err(rc);
		}
		n->fh = fh;
	}

	n->refs++;
	return 0;
}


int nfs_ops_close(nfs_fs_t *fs, oid_t *oid)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	if (n->refs > 0) {
		n->refs--;
	}

	/* Last close of an unlinked-while-open file (detached from byPath): the node
	 * can never be reused by name, so don't park it on the idle LRU (which never
	 * frees nodes). Close its orphaned fh and remove it outright. */
	if ((n->refs == 0) && (n->pathDetached != 0)) {
		if (n->fh != NULL) {
			nfs_close(fs->nfs, n->fh);
			n->fh = NULL;
		}
		nfs_node_remove(&fs->nodes, n);
		return 0;
	}

	/* Lazy-close (#156): on the last close, do NOT nfs_close the fh. Park the
	 * node on the idle LRU so the next open reuses the open fh (collapsing the
	 * loader's per-page open/close RPC pair). Only an over-cap eviction, an
	 * unlink, or a destroy actually nfs_close()s a cached fh. */
	if ((n->refs == 0) && (n->fh != NULL) && (n->idle == 0)) {
		nfs_node_idlePush(&fs->nodes, n);

		/* Bound the cached-open fhs: evict the LRU tail when over cap. */
		if (fs->nodes.idleCount > NFS_IDLE_MAX) {
			nfs_node_t *lru = nfs_node_idleLru(&fs->nodes);
			if ((lru != NULL) && (lru != n)) {
				if (lru->fh != NULL) {
					nfs_close(fs->nfs, lru->fh);
					lru->fh = NULL;
				}
				nfs_node_idleUnlink(&fs->nodes, lru);
			}
		}
	}

	return 0;
}


int nfs_ops_read(nfs_fs_t *fs, oid_t *oid, off_t offs, void *buf, size_t len)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -EINVAL;
	}
	if (offs < 0) {
		return -EINVAL;
	}
	if (len == 0) {
		return 0;
	}

	if (n->type == otSymlink) {
		/* No mtReadlink op: a read of an otSymlink object returns the target.
		 *
		 * libnfs 6.0.2 readlink_cb has a bug: when the target is longer than the
		 * caller's buffer it records -ENAMETOOLONG but cb_data_is_finished()
		 * immediately overwrites status with the RPC success code, so nfs_readlink
		 * returns phantom success with the buffer left UNMODIFIED. Reading straight
		 * into the (possibly small) client buffer would then return strnlen() over
		 * stale bytes -> a bogus target -> realpath/readlink of any over-long-target
		 * symlink resolves wrong (observed: ENOENT instead of a truncated result).
		 *
		 * Stage into a full PATH_MAX buffer so libnfs never takes that branch (a
		 * valid symlink target is <= PATH_MAX), then apply POSIX readlink truncation
		 * ourselves: copy min(target_len, len) and return that count. A caller whose
		 * buffer is too small thus gets a returned count == its buffer size, which is
		 * the truncation signal POSIX (and libphoenix realpath) expects. */
		char *tmp = malloc(PATH_MAX + 1);
		if (tmp == NULL) {
			return -ENOMEM;
		}
		int rc = nfs_readlink(fs->nfs, n->path, tmp, PATH_MAX + 1);
		if (rc != 0) {
			free(tmp);
			return nfs_err(rc);
		}
		size_t tlen = strnlen(tmp, PATH_MAX + 1);
		size_t cpy = (tlen < len) ? tlen : len;
		memcpy(buf, tmp, cpy);
		free(tmp);
		return (int)cpy;
	}

	if (n->type == otDir) {
		return -EISDIR;
	}

	/* Regular file. Wrap "ensure an fh + read" in a reclaim loop: an NFSv4
	 * lease/state expiry (idle >~90s, or server state loss) surfaces here — the
	 * exec loader demand-pages a binary in through this path, so a cached open
	 * stateid can die between pages. On expiry we re-establish client state and
	 * retry; the read itself is otherwise unchanged. */
	int reclaimBudget = NFS_RECLAIM_MAX;
	for (;;) {
		struct nfsfh *fh = n->fh;
		int owned = 0;
		if (fh == NULL) {
			/* Open-on-demand by path (bounded retry on transient RPC errors, same
			 * rationale as the read below). */
			int rc = -EIO;
			for (int tries = 0; tries < 25; tries++) {
				rc = nfs_open(fs->nfs, n->path, O_RDONLY, &fh);
				if (rc == 0) {
					break;
				}
				int e = nfs_err(rc);
				if (e != -EIO && e != -ETIMEDOUT) {
					break;
				}
				usleep(tries < 6 ? (10000u << tries) : 640000u);
			}
			if (rc != 0) {
				if (nfs_tryReclaim(fs, rc, &reclaimBudget) != 0) {
					continue;
				}
				return nfs_err(rc);
			}
			owned = 1;
		}

		/* Bounded retry on transient RPC errors. A single connection reset / timeout mid-transfer
		 * otherwise fails the whole read; for exec-over-NFS of a large binary (17MB rpi4-quake =
		 * thousands of demand-paged reads) that surfaced as an intermittent `exec ... failed (-5)`.
		 * libnfs reconnects on the next call, so retry with backoff. Non-transient errors (ENOENT,
		 * EISDIR, ...) break immediately. */
		int rc = -EIO;
		for (int tries = 0; tries < 25; tries++) {
			rc = nfs_pread(fs->nfs, fh, buf, len, offs);
			if (rc >= 0) {
				break;
			}
			int e = nfs_err(rc);
			if (e != -EIO && e != -ETIMEDOUT) {
				break;
			}
			usleep(tries < 6 ? (10000u << tries) : 640000u);   /* 10,20,40,80,160,320,640ms... */
		}

		if (rc >= 0) {
			if (owned != 0) {
				nfs_close(fs->nfs, fh);
			}
			return rc;
		}

		/* Read failed. If the client state expired, reclaim and retry: do NOT
		 * nfs_close(fh) here — on expiry the fh is dead and reclaim destroys the
		 * whole context (freeing it); a cached n->fh is invalidated by reclaim so
		 * the next pass re-opens by path. */
		if (nfs_tryReclaim(fs, rc, &reclaimBudget) != 0) {
			continue;
		}

		if (owned != 0) {
			nfs_close(fs->nfs, fh);
		}
		return nfs_err(rc);
	}
}


int nfs_ops_write(nfs_fs_t *fs, oid_t *oid, off_t offs, const void *buf, size_t len)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -EINVAL;
	}
	if (offs < 0) {
		return -EINVAL;
	}
	if (len == 0) {
		return 0;
	}
	if (n->type == otDir) {
		return -EISDIR;
	}

	/* Reclaim loop (see nfs_ops_read): re-establish NFSv4 client state and retry
	 * if the lease/state expired under us. */
	int reclaimBudget = NFS_RECLAIM_MAX;
	for (;;) {
		struct nfsfh *fh = n->fh;
		int owned = 0;
		if (fh == NULL) {
			int rc = nfs_open(fs->nfs, n->path, O_RDWR, &fh);
			if (rc != 0) {
				if (nfs_tryReclaim(fs, rc, &reclaimBudget) != 0) {
					continue;
				}
				return nfs_err(rc);
			}
			owned = 1;
		}

		int rc = nfs_pwrite(fs->nfs, fh, (void *)buf, len, offs);

		if (rc >= 0) {
			if (owned != 0) {
				nfs_close(fs->nfs, fh);
			}
			return rc;
		}

		/* On expiry, do not nfs_close(fh) — reclaim frees the whole context. */
		if (nfs_tryReclaim(fs, rc, &reclaimBudget) != 0) {
			continue;
		}

		if (owned != 0) {
			nfs_close(fs->nfs, fh);
		}
		return nfs_err(rc);
	}
}


int nfs_ops_truncate(nfs_fs_t *fs, oid_t *oid, size_t size)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -EINVAL;
	}

	int rc;
	if (n->fh != NULL) {
		rc = nfs_ftruncate(fs->nfs, n->fh, size);
	}
	else {
		rc = nfs_truncate(fs->nfs, n->path, size);
	}

	return (rc != 0) ? nfs_err(rc) : 0;
}


/* POSIX st_blocks is the number of 512-byte units actually allocated. Defensive
 * fallback: if the server/libnfs does not report an allocated block count
 * (nfs_blocks == 0, e.g. some exports) approximate it from the file size so
 * du(1)/ls -s do not show 0. NOTE: when libnfs DOES report nfs_blocks it appears
 * to be in the NFS block size (nfs_blksize, typically 4096) rather than 512-byte
 * units, so du can under-report ~8x -- a separate fix (nfs_blocks * nfs_blksize /
 * 512, after confirming libnfs's unit) is TODO. */
static long long nfs_blocksOf(const struct nfs_stat_64 *st)
{
	if (st->nfs_blocks != 0) {
		return (long long)st->nfs_blocks;
	}
	return (long long)(((unsigned long long)st->nfs_size + 511ULL) / 512ULL);
}


int nfs_ops_getattr(nfs_fs_t *fs, oid_t *oid, int type, long long *attr)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	struct nfs_stat_64 st;
	int rc = nfs_refreshStat(fs, n, &st);
	if (rc != 0) {
		return rc;
	}

	switch (type) {
		case atMode:
			*attr = (long long)st.nfs_mode;
			break;
		case atUid:
			*attr = (long long)st.nfs_uid;
			break;
		case atGid:
			*attr = (long long)st.nfs_gid;
			break;
		case atSize:
			*attr = (long long)st.nfs_size;
			break;
		case atBlocks:
			*attr = nfs_blocksOf(&st);
			break;
		case atIOBlock:
			*attr = (long long)st.nfs_blksize;
			break;
		case atType:
			*attr = n->type;
			break;
		case atCTime:
			*attr = (long long)st.nfs_ctime;
			break;
		case atMTime:
			*attr = (long long)st.nfs_mtime;
			break;
		case atATime:
			*attr = (long long)st.nfs_atime;
			break;
		case atLinks:
			*attr = (long long)st.nfs_nlink;
			break;
		case atPollStatus:
			*attr = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


int nfs_ops_getattrAll(nfs_fs_t *fs, oid_t *oid, struct _attrAll *attrs)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	struct nfs_stat_64 st;
	int rc = nfs_refreshStat(fs, n, &st);
	if (rc != 0) {
		return rc;
	}

	_phoenix_initAttrsStruct(attrs, -ENOSYS);

	attrs->mode.val = (long long)st.nfs_mode;
	attrs->mode.err = EOK;
	attrs->uid.val = (long long)st.nfs_uid;
	attrs->uid.err = EOK;
	attrs->gid.val = (long long)st.nfs_gid;
	attrs->gid.err = EOK;
	attrs->size.val = (long long)st.nfs_size;
	attrs->size.err = EOK;
	attrs->blocks.val = nfs_blocksOf(&st);
	attrs->blocks.err = EOK;
	attrs->ioblock.val = (long long)st.nfs_blksize;
	attrs->ioblock.err = EOK;
	attrs->type.val = n->type;
	attrs->type.err = EOK;
	attrs->cTime.val = (long long)st.nfs_ctime;
	attrs->cTime.err = EOK;
	attrs->mTime.val = (long long)st.nfs_mtime;
	attrs->mTime.err = EOK;
	attrs->aTime.val = (long long)st.nfs_atime;
	attrs->aTime.err = EOK;
	attrs->links.val = (long long)st.nfs_nlink;
	attrs->links.err = EOK;
	attrs->pollStatus.val = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
	attrs->pollStatus.err = EOK;

	return 0;
}


int nfs_ops_setattr(nfs_fs_t *fs, oid_t *oid, int type, long long val, const void *data, size_t size)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	int rc = 0;
	switch (type) {
		case atMode:
			rc = nfs_chmod(fs->nfs, n->path, (int)(val & ALLPERMS));
			break;

		case atSize:
			if (n->fh != NULL) {
				rc = nfs_ftruncate(fs->nfs, n->fh, (uint64_t)val);
			}
			else {
				rc = nfs_truncate(fs->nfs, n->path, (uint64_t)val);
			}
			break;

		case atMTime:
		case atATime:
			/* libnfs nfs_utimes takes both times; we only have one here, so
			 * stat the other and write both back. */
			{
				struct nfs_stat_64 st;
				if (nfs_lstat64(fs->nfs, n->path, &st) != 0) {
					return -EIO;
				}
				struct timeval tv[2];
				tv[0].tv_sec = (type == atATime) ? (time_t)val : (time_t)st.nfs_atime;
				tv[0].tv_usec = 0;
				tv[1].tv_sec = (type == atMTime) ? (time_t)val : (time_t)st.nfs_mtime;
				tv[1].tv_usec = 0;
				rc = nfs_utimes(fs->nfs, n->path, tv);
			}
			break;

		case atDev:
			/* Inbound mount-splice: a child fs mounts onto one of OUR dirs.
			 * Store the child's oid on the node (mirrors dummyfs's per-object
			 * `dev`); lookup then hands it back so the kernel redirects path
			 * resolution into the child fs. This is the mechanism that makes
			 * `bind devfs /dev` work once the NFS export owns "/" (#153 T3
			 * design-A) — without it the re-bind would silently no-op and /dev
			 * on the NFS root would be empty. */
			if ((data == NULL) || (size != sizeof(oid_t)) || (n->type != otDir)) {
				return -EINVAL;
			}
			memcpy(&n->mnt, data, sizeof(oid_t));
			return 0;

		default:
			return -EINVAL;
	}

	return (rc != 0) ? nfs_err(rc) : 0;
}


int nfs_ops_create(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *res, unsigned mode, int type, oid_t *dev)
{
	(void)dev; /* device oid only meaningful for block/char nodes, which we mknod with dev=0 */
	nfs_node_t *d = nfs_node_find(&fs->nodes, dir->id);
	if (d == NULL) {
		return -ENOENT;
	}

	/* mtCreate carries the bare name (and for symlinks, name\0target\0). */
	char *path = nfs_node_joinPath(d->path, name);
	if (path == NULL) {
		return -ENOMEM;
	}

	int rc = 0;
	switch (type) {
		case otFile: {
			struct nfsfh *fh = NULL;
			rc = nfs_creat(fs->nfs, path, (int)(mode & ALLPERMS), &fh);
			if (rc == 0) {
				if (fh != NULL) {
					nfs_close(fs->nfs, fh);
				}
				/* HW-observed: nfs_creat over NFSv4 leaves mode 000 on the
				 * server. Force the requested perms so executables get +x. */
				(void)nfs_chmod(fs->nfs, path, (int)(mode & ALLPERMS));
			}
			break;
		}

		case otDir:
			rc = nfs_mkdir2(fs->nfs, path, (int)(mode & ALLPERMS));
			break;

		case otSymlink: {
			/* symlink(): two strings, the second is the target (see dummyfs). */
			const char *target = name + strlen(name) + 1;
			rc = nfs_symlink(fs->nfs, target, path);
			break;
		}

		case otDev:
			rc = nfs_mknod(fs->nfs, path, (int)mode, 0);
			break;

		default:
			free(path);
			return -EINVAL;
	}

	if (rc != 0) {
		free(path);
		return nfs_err(rc);
	}

	nfs_node_t *n = nfs_node_get(&fs->nodes, path);
	free(path);
	if (n == NULL) {
		return -ENOMEM;
	}
	n->type = (type == otDir) ? otDir : ((type == otSymlink) ? otSymlink : ((type == otDev) ? otDev : otFile));

	res->port = fs->port;
	res->id = n->id;

	return 0;
}


int nfs_ops_destroy(nfs_fs_t *fs, oid_t *oid)
{
	nfs_node_t *n = nfs_node_find(&fs->nodes, oid->id);
	if (n == NULL) {
		return -ENOENT;
	}

	if (n->fh != NULL) {
		nfs_close(fs->nfs, n->fh);
		n->fh = NULL;
	}
	nfs_node_remove(&fs->nodes, n);

	return 0;
}


int nfs_ops_unlink(nfs_fs_t *fs, oid_t *dir, const char *name)
{
	if ((name == NULL) || (strcmp(name, ".") == 0) || (strcmp(name, "..") == 0)) {
		return -EINVAL;
	}

	nfs_node_t *d = nfs_node_find(&fs->nodes, dir->id);
	if (d == NULL) {
		return -EINVAL;
	}

	char *path = nfs_node_joinPath(d->path, name);
	if (path == NULL) {
		return -ENOMEM;
	}

	/* don't-follow stat: a symlink-to-a-dir is otSymlink -> nfs_unlink. */
	struct nfs_stat_64 st;
	int rc = nfs_lstat64(fs->nfs, path, &st);
	if (rc != 0) {
		free(path);
		return -ENOENT;
	}

	if (S_ISDIR(st.nfs_mode)) {
		rc = nfs_rmdir(fs->nfs, path);
	}
	else {
		rc = nfs_unlink(fs->nfs, path);
	}

	/* Drop any cached node for this path (find-only: don't materialize one). */
	nfs_node_t *n = nfs_node_findPath(&fs->nodes, path);
	free(path);
	if ((rc == 0) && (n != NULL)) {
		if (n->refs == 0) {
			/* A node at refs==0 may still hold a lazily-cached fh on the idle LRU
			 * (#156); close it before removing so the fh isn't leaked. */
			if (n->fh != NULL) {
				nfs_close(fs->nfs, n->fh);
				n->fh = NULL;
			}
			nfs_node_remove(&fs->nodes, n);
		}
		else {
			/* Still open: the file is gone from the directory but open fds keep it
			 * alive server-side (POSIX unlink-while-open). Don't touch n->fh (the
			 * fds need it) — just unbind the name so a later create/lookup of the
			 * same path mints a FRESH node instead of aliasing this node's now-
			 * orphaned fh (which would split writes and stat across two inodes). */
			nfs_node_detachPath(&fs->nodes, n);
		}
	}

	return (rc != 0) ? nfs_err(rc) : 0;
}


int nfs_ops_link(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *oid)
{
	nfs_node_t *d = nfs_node_find(&fs->nodes, dir->id);
	nfs_node_t *target = nfs_node_find(&fs->nodes, oid->id);
	if ((d == NULL) || (target == NULL)) {
		return -ENOENT;
	}

	char *path = nfs_node_joinPath(d->path, name);
	if (path == NULL) {
		return -ENOMEM;
	}

	int rc = nfs_link(fs->nfs, target->path, path);
	free(path);

	return (rc != 0) ? nfs_err(rc) : 0;
}


int nfs_ops_readdir(nfs_fs_t *fs, oid_t *dir, off_t offs, struct dirent *dent, size_t size)
{
	nfs_node_t *d = nfs_node_find(&fs->nodes, dir->id);
	if (d == NULL) {
		return -ENOENT;
	}
	if (offs < 0) {
		return -EINVAL;
	}

	/* Snapshot strategy: open the dir, walk to the cumulative-name-length
	 * cookie (matching dummyfs semantics where the cookie is sum of name
	 * lengths, d_reclen == name length), emit that one entry, close. */
	struct nfsdir *nfsdir = NULL;
	int rc = nfs_opendir(fs->nfs, d->path, &nfsdir);
	if (rc != 0) {
		return nfs_err(rc);
	}

	struct nfsdirent *ent;
	int emitted = -ENOENT;

	/* POSIX readdir must return "." and ".." — NFS READDIR (via libnfs) does not
	 * include them, so synthesize them as the first two entries (matching dummyfs).
	 * With the cumulative-name-length cookie ("."=1, ".."=2) they consume cookie
	 * slots 0 and 1, so the libnfs entries below start at diroffs 3. */
	if ((offs == 0) || (offs == 1)) {
		const char *dot = (offs == 0) ? "." : "..";
		size_t namelen = (offs == 0) ? 1 : 2;
		/* Resolve the real NFS inode so "." (this dir) and ".." (its parent) carry
		 * correct, mutually-distinct inode numbers (the server resolves the trailing
		 * ".." for us) rather than both reusing d->id. Fall back to d->id on error. */
		ino_t ino = (ino_t)d->id;
		struct nfs_stat_64 st;
		char qpath[512];
		int qn = (offs == 0) ? snprintf(qpath, sizeof(qpath), "%s", d->path)
		                     : snprintf(qpath, sizeof(qpath), "%s/..", d->path);
		if ((qn > 0) && ((size_t)qn < sizeof(qpath)) && (nfs_lstat64(fs->nfs, qpath, &st) == 0)) {
			ino = (ino_t)st.nfs_ino;
		}
		dent->d_ino = ino;
		dent->d_reclen = (uint16_t)namelen;
		dent->d_namlen = (uint16_t)namelen;
		dent->d_type = otDir;
		memcpy(dent->d_name, dot, namelen);
		dent->d_name[namelen] = '\0';
		nfs_closedir(fs->nfs, nfsdir);
		return 0;
	}

	off_t diroffs = 3;
	while ((ent = nfs_readdir(fs->nfs, nfsdir)) != NULL) {
		size_t namelen = strlen(ent->name);

		/* skip any server-provided "."/".." so the synthesized ones are not duplicated */
		if ((ent->name[0] == '.') && ((namelen == 1) || ((namelen == 2) && (ent->name[1] == '.')))) {
			continue;
		}

		if (diroffs >= offs) {
			if ((sizeof(struct dirent) + namelen + 1) > size) {
				emitted = -EINVAL;
				break;
			}
			dent->d_ino = (ino_t)ent->inode;
			dent->d_reclen = (uint16_t)namelen;
			dent->d_namlen = (uint16_t)namelen;

			/* Map libnfs NF3* dir-entry type to Phoenix otX. type==0 (no
			 * READDIRPLUS attrs) falls through to otUnknown. */
			if (S_ISDIR(ent->mode)) {
				dent->d_type = otDir;
			}
			else if (S_ISLNK(ent->mode)) {
				dent->d_type = otSymlink;
			}
			else if (S_ISCHR(ent->mode) || S_ISBLK(ent->mode) || S_ISFIFO(ent->mode)) {
				dent->d_type = otDev;
			}
			else if (S_ISREG(ent->mode)) {
				dent->d_type = otFile;
			}
			else {
				dent->d_type = otUnknown;
			}

			memcpy(dent->d_name, ent->name, namelen);
			dent->d_name[namelen] = '\0';
			emitted = 0;
			break;
		}

		diroffs += (off_t)namelen;
	}

	nfs_closedir(fs->nfs, nfsdir);

	return emitted;
}


int nfs_ops_statfs(nfs_fs_t *fs, void *buf, size_t len)
{
	struct statvfs *st = buf;
	if ((st == NULL) || (len != sizeof(*st))) {
		return -EINVAL;
	}

	struct nfs_statvfs_64 nst;
	memset(&nst, 0, sizeof(nst));
	int rc = nfs_statvfs64(fs->nfs, "/", &nst);
	if (rc != 0) {
		return nfs_err(rc);
	}

	memset(st, 0, sizeof(*st));
	st->f_bsize = nst.f_bsize;
	st->f_frsize = nst.f_frsize;
	st->f_blocks = nst.f_blocks;
	st->f_bfree = nst.f_bfree;
	st->f_bavail = nst.f_bavail;
	st->f_files = nst.f_files;
	st->f_ffree = nst.f_ffree;
	st->f_favail = nst.f_favail;
	st->f_namemax = nst.f_namemax;

	return 0;
}

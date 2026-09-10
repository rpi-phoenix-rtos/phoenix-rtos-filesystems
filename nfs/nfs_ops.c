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
#include <time.h>     /* clock_gettime (attribute-cache deadlines) */
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


/* How long a node's don't-follow stat may be reused. Path resolution issues a
 * burst of lookups for the SAME prefixes within a few milliseconds (see the
 * attribute-cache comment in nfs_node.h), so a very short window already removes
 * the quadratic term. Keep it short: it is the only interval in which a change
 * made directly on the server can be invisible here. For reference, a Linux NFS
 * client caches regular-file attributes for 3-60 s (acregmin/acregmax). */
#define NFS_ATTR_TTL_US 100000u


/* CLOCK_MONOTONIC in microseconds, or 0 if unavailable (treated as "no cache"). */
static uint64_t nfs_nowUs(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		return 0;
	}
	return ((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL);
}


static int nfs_attrFresh(const nfs_node_t *n)
{
	if (n->attrValid == 0) {
		return 0;
	}

	uint64_t now = nfs_nowUs();

	return ((now != 0) && (now < n->attrValid)) ? 1 : 0;
}


static void nfs_attrStore(nfs_node_t *n, const struct nfs_stat_64 *st)
{
	uint64_t now = nfs_nowUs();

	if (now == 0) {
		n->attrValid = 0;
		return;
	}
	n->attr = *st;
	n->attrValid = now + NFS_ATTR_TTL_US;
}


/* Forget what we cached about a node — call from every op that changes the
 * object (or its name) so the next stat goes back to the server. */
static void nfs_attrDrop(nfs_node_t *n)
{
	if (n != NULL) {
		n->attrValid = 0;
	}
}


/* Close a node's cached directory snapshot, if it holds one. Safe to call on a
 * node that never had one, and on the node currently registered as fs->scanNode.
 *
 * Clears scanNode BEFORE the "nothing cached" early return, so this is the one
 * authority on the pairing rather than relying on every caller to have kept
 * scanNode and dirCache consistent. */
static void nfs_dirDrop(nfs_fs_t *fs, nfs_node_t *n)
{
	if (n == NULL) {
		return;
	}

	if (fs->scanNode == n) {
		fs->scanNode = NULL;
	}

	if (n->dirCache == NULL) {
		return;
	}

	nfs_closedir(fs->nfs, n->dirCache);
	n->dirCache = NULL;
	n->dirOffs = 0;
}


/* Refresh a node's cached type via a don't-follow stat. Returns 0 or -errno. Bounded retry on
 * transient RPC errors (contributes to the intermittent exec -5: the mtOpen path stats first).
 *
 * Answered from the node's attribute cache unless `force` is set; a fresh stat
 * repopulates it. Pass force != 0 where the point of the call IS to observe the
 * server (mtOpen's redeploy check). */
static int nfs_refreshStat(nfs_fs_t *fs, nfs_node_t *n, struct nfs_stat_64 *st, int force)
{
	int rc = -EIO;

	/* A special file spliced in from another server (see the otDev case in
	 * nfs_ops_create) exists only in this table, so stat'ing the export would
	 * answer ENOENT for a name that callers can open perfectly well. Synthesise
	 * it from what mkfifo()/mknod() asked for. */
	if ((n->mnt.port != 0) && (n->type == otDev)) {
		memset(st, 0, sizeof(*st));
		st->nfs_mode = n->mode;
		st->nfs_nlink = 1;
		st->nfs_ino = (uint64_t)n->id;
		return 0;
	}

	/* NOTE: below the spliced-special-file branch on purpose — that node has no
	 * file on the export, so it must always be answered by synthesis. */
	if ((force == 0) && (nfs_attrFresh(n) != 0)) {
		*st = n->attr;
		n->type = nfs_typeFromMode(st->nfs_mode);
		return 0;
	}

	for (int tries = 0; tries < 25; tries++) {
		rc = nfs_lstat64(fs->nfs, n->path, st);
		if ((rc == 0) || !nfs_transient(rc)) {
			break;
		}
		usleep(tries < 6 ? (10000u << tries) : 640000u);
	}
	if (rc != 0) {
		nfs_attrDrop(n);
		return nfs_err(rc);
	}
	n->type = nfs_typeFromMode(st->nfs_mode);
	nfs_attrStore(n, st);
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

	/* Close any open directory snapshot through the context that owns it, while
	 * we still have that context. nfs_closedir with the dircache disabled (see
	 * srv.c) lands in nfs_free_nfsdir, which ignores its nfs argument entirely,
	 * so this is safe here — and it is the only thing that frees the listing:
	 * nfs_destroy_context only walks the dircache list, which we keep empty, so
	 * dropping the pointer instead would strand the snapshot and every strdup'd
	 * name in it (tens of KB for a large directory) on every reclaim. */
	nfs_dirDrop(fs, fs->scanNode);

	struct nfs_context *old = fs->nfs;
	fs->nfs = fresh;
	/* The cached filehandles belong to the context about to be destroyed and
	 * their open stateids are dead, so they are forgotten structurally rather
	 * than closed. */
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

		/* A component may be a locally-spliced foreign object -- a mounted child
		 * fs, or a FIFO/device node whose owner is another server (see the
		 * otDev case in nfs_ops_create). Those have NO file on the export, so
		 * the stat below would correctly answer ENOENT. Resolve them from the
		 * local table first and hand back the owner's oid; the kernel then
		 * re-resolves any remainder against that server. */
		nfs_node_t *ln = nfs_node_findPath(&fs->nodes, child);
		if ((ln != NULL) && (ln->mnt.port != 0)) {
			free(child);
			if (ln->type == otDev) {
				/* Special file: the NODE is ours (stat must reach us), only
				 * opens go to the owner. Same split dummyfs uses -- handing
				 * back the foreign oid for both made stat() return EINVAL,
				 * because posixsrv was being asked to describe a file. */
				res->port = fs->port;
				res->id = ln->id;
				*dev = ln->mnt;
			}
			else {
				/* Mountpoint: the child fs owns everything below here. */
				*res = ln->mnt;
				*dev = ln->mnt;
			}
			return len + comp;
		}

		struct nfs_stat_64 st;
		int rc = 0;
		int cached = 0;
		/* Already described this component recently? Resolving one absolute path
		 * walks its prefixes repeatedly (see nfs_node.h), so nearly every
		 * component below the leaf is answered from here rather than the wire. */
		if ((ln != NULL) && (nfs_attrFresh(ln) != 0)) {
			st = ln->attr;
			cached = 1;
		}
		else {
			/* Bounded retry on transient RPC errors (connection reset/timeout); a genuine missing
			 * entry (ENOENT) breaks immediately. Otherwise a transient stat failure during path
			 * resolution fails the whole open/exec (contributes to the intermittent exec -5). */
			rc = -EIO;
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
		}
		if (rc != 0) {
			nfs_attrDrop(ln);
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
		/* Only a stat we just took extends the window — re-stamping a cache hit
		 * would keep one entry alive indefinitely under a repeated lookup. */
		if (cached == 0) {
			nfs_attrStore(cn, &st);
		}

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

	/* Re-stat on open so a redeployed file isn't shadowed (OQ-B) — force, since
	 * answering this from the attribute cache would defeat its whole purpose. */
	struct nfs_stat_64 st;
	int rc = nfs_refreshStat(fs, n, &st, 1);
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
		/* A directory removed while a scan held it open reaches here too, and the
		 * node is about to be freed — the snapshot must not outlive it. */
		nfs_dirDrop(fs, n);
		nfs_node_remove(&fs->nodes, n);
		return 0;
	}

	/* A directory never gets an fh (nfs_ops_open opens one only for otFile), so
	 * neither branch below covers it: an abandoned or partial scan would pin its
	 * whole listing until some other readdir happened to evict it. Release it
	 * here, where the owner actually says it is done. */
	if ((n->refs == 0) && (n->type == otDir)) {
		nfs_dirDrop(fs, n);
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
			nfs_attrDrop(n); /* size/mtime moved */
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
	nfs_attrDrop(n); /* size/mtime moved */

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
	int rc = nfs_refreshStat(fs, n, &st, 0);
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
	int rc = nfs_refreshStat(fs, n, &st, 0);
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
	/* Every branch below changes something a stat reports (mode, size, times) or
	 * what the node even is (atDev), so nothing cached about it survives. */
	nfs_attrDrop(n);

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
				/* Direct lstat, not nfs_refreshStat: we are about to write these
				 * timestamps back, so they must be the server's current ones. */
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
			/* A device or FIFO node. `dev` is the oid of the server that owns
			 * the object, and that binding is the entire point: opens of this
			 * name must reach THAT server. nfs_mknod() cannot express it, and
			 * over NFSv4 it fails outright -- which is how every mkfifo() on an
			 * NFS root came back EIO (posix_mkfifo creates the pipe in posixsrv
			 * and then asks the owning filesystem for an otDev node carrying its
			 * oid; libc/stdio's wrong_stream_type_fifo has been failing on
			 * exactly this). So do not touch the server: record the name locally
			 * and splice the foreign oid in below, the same way lookup already
			 * resolves a mounted child (node->mnt).
			 *
			 * Consequence to know about: the name lives in this mount only, so a
			 * readdir on the export does not list it and it does not survive a
			 * remount. That matches what the object is -- a pipe in another
			 * process, which could not be reconstructed from the server anyway. */
			rc = 0;
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
	nfs_attrDrop(d); /* directory mtime/nlink moved */
	nfs_attrDrop(n); /* a name reused after an unlink must not inherit the old stat */
	nfs_dirDrop(fs, d); /* ... and its listing no longer has every name in it */
	n->type = (type == otDir) ? otDir : ((type == otSymlink) ? otSymlink : ((type == otDev) ? otDev : otFile));
	if ((type == otDev) && (dev != NULL)) {
		n->mnt = *dev;   /* opens of this name go to the owning server */
		n->mode = mode;  /* ... but WE answer stat: there is no file to stat */
	}

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
	nfs_dirDrop(fs, n);
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

	/* A locally-spliced foreign object (FIFO/device from another server) has no
	 * file on the export: drop the local binding and stop, or the stat below
	 * would report ENOENT for a name that does exist as far as callers see. */
	{
		nfs_node_t *ln = nfs_node_findPath(&fs->nodes, path);

		if ((ln != NULL) && (ln->mnt.port != 0)) {
			free(path);
			nfs_dirDrop(fs, ln);
			nfs_node_remove(&fs->nodes, ln);
			return 0;
		}
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

	nfs_attrDrop(d); /* directory mtime/nlink moved */
	nfs_dirDrop(fs, d); /* ... and its listing still has the removed name in it */

	/* st_nlink moved for EVERY name of that inode, not just this one. Dropping
	 * only the unlinked node (below) left a sibling hard link answering stat()
	 * from its own cache with the pre-unlink count for up to NFS_ATTR_TTL_US --
	 * a wrong answer, and an intermittent one, since whether it shows depends on
	 * where the 100 ms TTL happens to fall. nfs_ops_link() already dropped the
	 * target for exactly this reason ("st_nlink moved"); unlink needs the
	 * matching sweep. `st` is the pre-unlink don't-follow stat above, so
	 * st.nfs_ino is the inode whose count just changed. */
	if (rc == 0) {
		nfs_node_attrDropByIno(&fs->nodes, st.nfs_ino);
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
			nfs_dirDrop(fs, n);
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
	if (d == NULL) {
		return -ENOENT;
	}

	char *path = nfs_node_joinPath(d->path, name);
	if (path == NULL) {
		return -ENOMEM;
	}

	nfs_node_t *target = nfs_node_find(&fs->nodes, oid->id);
	if (target == NULL) {
		return -ENOENT;
	}

	/* Capture the inode before dropping, so the sibling sweep below can still
	 * see which one it was. */
	uint64_t ino = (target->attrValid != 0) ? target->attr.nfs_ino : 0;

	int rc = nfs_link(fs->nfs, target->path, path);
	free(path);
	nfs_attrDrop(d);      /* directory mtime moved */
	nfs_attrDrop(target); /* st_nlink moved */
	/* ... and it moved for the inode's OTHER names too. Only possible when we
	 * had the inode cached; with nothing cached there is nothing stale to drop
	 * for the named node anyway, and any sibling will have its own TTL. */
	if ((rc == 0) && (ino != 0)) {
		nfs_node_attrDropByIno(&fs->nodes, ino);
	}

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

	/* Snapshot strategy: walk to the cumulative-name-length cookie (matching
	 * dummyfs semantics where the cookie is the sum of name lengths and
	 * d_reclen == name length) and emit that one entry. The snapshot is kept
	 * open between calls (see dirCache in nfs_node.h) so a sequential scan pays
	 * for ONE listing rather than one per entry. */
	struct nfsdirent *ent;
	int emitted = -ENOENT;

	/* POSIX readdir must return "." and ".." — NFS READDIR (via libnfs) does not
	 * include them, so synthesize them as the first two entries (matching dummyfs).
	 * With the cumulative-name-length cookie ("."=1, ".."=2) they consume cookie
	 * slots 0 and 1, so the libnfs entries below start at diroffs 3. */
	if ((offs == 0) || (offs == 1)) {
		const char *dot = (offs == 0) ? "." : "..";
		size_t namelen = (offs == 0) ? 1 : 2;

		/* Bound this write like the main path below. The real client always
		 * passes sizeof(struct dirent) + NAME_MAX + 1, but dent/size come
		 * straight off the message, so a malformed mtReaddir must not get a
		 * write into a buffer we never measured. */
		if ((dent == NULL) || ((sizeof(struct dirent) + namelen + 1) > size)) {
			return -EINVAL;
		}

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
		/* offs 0 is a fresh scan (or a rewind): forget any snapshot so the entries
		 * this scan goes on to read are listed now, not inherited from last time. */
		if (offs == 0) {
			nfs_dirDrop(fs, d);
		}
		return 0;
	}

	/* Reuse the snapshot only if it is positioned exactly at the requested cookie;
	 * anything else (a seek, a second interleaved scan) re-lists. */
	struct nfsdir *nfsdir = NULL;
	off_t diroffs;

	if ((d->dirCache != NULL) && (d->dirOffs == offs)) {
		nfsdir = d->dirCache;
		diroffs = offs;
	}
	else {
		nfs_dirDrop(fs, d);
		/* One snapshot at a time across the whole fs, so a scan cannot pin an
		 * unbounded amount of listing memory. */
		nfs_dirDrop(fs, fs->scanNode);

		int rc = nfs_opendir(fs->nfs, d->path, &nfsdir);
		/* The handle is now persisted on the node, so a success return with a
		 * NULL dir would be dereferenced by a LATER call, not this one. */
		if ((rc != 0) || (nfsdir == NULL)) {
			return (rc != 0) ? nfs_err(rc) : -EIO;
		}
		d->dirCache = nfsdir;
		fs->scanNode = d;
		diroffs = 3;
	}

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
			/* Leave the snapshot open, positioned at the next entry, so the next
			 * call is a local step instead of another listing. */
			d->dirOffs = offs + (off_t)namelen;
			break;
		}

		diroffs += (off_t)namelen;
	}

	if (emitted != 0) {
		/* End of directory, or the caller's buffer was too small: either way the
		 * snapshot is no longer positioned anywhere useful. */
		nfs_dirDrop(fs, d);
	}

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

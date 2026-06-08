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
#include <poll.h>
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


/* Refresh a node's cached type via a don't-follow stat. Returns 0 or -errno. */
static int nfs_refreshStat(nfs_fs_t *fs, nfs_node_t *n, struct nfs_stat_64 *st)
{
	int rc = nfs_lstat64(fs->nfs, n->path, st);
	if (rc != 0) {
		return nfs_err(rc);
	}
	n->type = nfs_typeFromMode(st->nfs_mode);
	return 0;
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
		int rc = nfs_lstat64(fs->nfs, child, &st);
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
		struct nfsfh *fh = NULL;
		rc = nfs_open(fs->nfs, n->path, O_RDWR, &fh);
		if (rc != 0) {
			/* fall back to read-only (e.g. mode lacks write) */
			rc = nfs_open(fs->nfs, n->path, O_RDONLY, &fh);
			if (rc != 0) {
				return nfs_err(rc);
			}
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
				printf("nfs-fs: fh-cache evict, %u idle\n", fs->nodes.idleCount);
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
		/* No mtReadlink op: a read of an otSymlink object returns the target. */
		int rc = nfs_readlink(fs->nfs, n->path, buf, len);
		if (rc != 0) {
			return nfs_err(rc);
		}
		return (int)strnlen(buf, len);
	}

	if (n->type == otDir) {
		return -EISDIR;
	}

	/* Regular file: use the cached fh, else open-on-demand by path. */
	struct nfsfh *fh = n->fh;
	int owned = 0;
	if (fh == NULL) {
		int rc = nfs_open(fs->nfs, n->path, O_RDONLY, &fh);
		if (rc != 0) {
			return nfs_err(rc);
		}
		owned = 1;
	}

	int rc = nfs_pread(fs->nfs, fh, buf, len, offs);

	if (owned != 0) {
		nfs_close(fs->nfs, fh);
	}

	if (rc < 0) {
		return nfs_err(rc);
	}
	return rc;
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

	struct nfsfh *fh = n->fh;
	int owned = 0;
	if (fh == NULL) {
		int rc = nfs_open(fs->nfs, n->path, O_RDWR, &fh);
		if (rc != 0) {
			return nfs_err(rc);
		}
		owned = 1;
	}

	int rc = nfs_pwrite(fs->nfs, fh, (void *)buf, len, offs);

	if (owned != 0) {
		nfs_close(fs->nfs, fh);
	}

	if (rc < 0) {
		return nfs_err(rc);
	}
	return rc;
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
			*attr = (long long)st.nfs_blocks;
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
	attrs->blocks.val = (long long)st.nfs_blocks;
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
	if ((rc == 0) && (n != NULL) && (n->refs == 0)) {
		/* A node at refs==0 may still hold a lazily-cached fh on the idle LRU
		 * (#156); close it before removing so the fh isn't leaked. */
		if (n->fh != NULL) {
			nfs_close(fs->nfs, n->fh);
			n->fh = NULL;
		}
		nfs_node_remove(&fs->nodes, n);
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

	off_t diroffs = 0;
	struct nfsdirent *ent;
	int emitted = -ENOENT;

	while ((ent = nfs_readdir(fs->nfs, nfsdir)) != NULL) {
		size_t namelen = strlen(ent->name);

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

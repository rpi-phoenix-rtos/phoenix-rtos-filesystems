/*
 * Phoenix-RTOS — NFS filesystem server
 *
 * Directory listing snapshot with readdir positions that survive removals.
 *
 * A scan that removes what it reads (rm -rf, find -delete) must find the
 * remaining entries where it left them. NFS READDIR cookies would give that,
 * but libnfs does not expose them: struct nfsdirent carries no cookie,
 * nfs_opendir() always lists from cookie 0, and nfs_telldir()/nfs_seekdir()
 * are indexes into its own list, which shift when an entry goes away.
 *
 * So the position of an entry is derived from its name (nfs_dir_pos()) and the
 * snapshot is kept in position order. A position then means the same entry in
 * any listing of the directory, including one made after other entries were
 * removed. Two names whose hashes agree share a position, and a scan returns
 * only the first of them; with 62 hash bits that takes ~2^31 names in one
 * directory to become likely.
 *
 * Kept free of libnfs and of the message ABI so it can be tested on the host.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#ifndef _NFS_DIR_H_
#define _NFS_DIR_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>


/* "." and ".." are positions 0 and 1; listed entries start at NFS_DIR_FIRST. */
#define NFS_DIR_FIRST 3


typedef struct {
	off_t pos;      /* stable position (nfs_dir_pos() of the name) */
	off_t lpos;     /* position for a client without MSG_READDIR_NEXT, see nfs_dir_seekLegacy() */
	uint64_t ino;
	int type;       /* otDir/otFile/... */
	int gone;       /* removed through this server since the listing was made */
	size_t namelen;
	char *name;
} nfs_dirEntry_t;


typedef struct nfs_dirSnap {
	nfs_dirEntry_t *ents; /* sorted by pos, then name */
	size_t n;
	size_t cap;
} nfs_dirSnap_t;


/* Position of the entry called name. Always >= NFS_DIR_FIRST, and the position
 * after it (pos + 1) still fits in a positive off_t. */
extern off_t nfs_dir_pos(const char *name, size_t len);


/* Allocate an empty snapshot. Returns NULL on OOM. */
extern nfs_dirSnap_t *nfs_dir_new(void);


/* Free a snapshot and its names. NULL is a no-op. */
extern void nfs_dir_free(nfs_dirSnap_t *s);


/* Append an entry while building. Returns 0 or -ENOMEM. */
extern int nfs_dir_add(nfs_dirSnap_t *s, const char *name, uint64_t ino, int type);


/* Put the entries in position order. Call once after the last nfs_dir_add(). */
extern void nfs_dir_sort(nfs_dirSnap_t *s);


/* Mark the entry called name as removed (a no-op if there is none). */
extern void nfs_dir_remove(nfs_dirSnap_t *s, const char *name);


/* First live entry whose position is >= offs, or NULL at the end. */
extern const nfs_dirEntry_t *nfs_dir_seek(const nfs_dirSnap_t *s, off_t offs);


/* For a client that advances by d_reclen (no MSG_READDIR_NEXT): positions are
 * NFS_DIR_FIRST plus the summed name lengths of the entries before, removed
 * entries included, so they hold while this snapshot lives. Returns the first
 * live entry at or after offs and stores the position after it in *end. */
extern const nfs_dirEntry_t *nfs_dir_seekLegacy(const nfs_dirSnap_t *s, off_t offs, off_t *end);


/* The entry a readdir at offs returns (offs >= NFS_DIR_FIRST - 1), or NULL at
 * the end; *end receives the position after it. stable != 0 for a client that
 * takes that position back (MSG_READDIR_NEXT), else the d_reclen layout. */
extern const nfs_dirEntry_t *nfs_dir_read(const nfs_dirSnap_t *s, off_t offs, int stable, off_t *end);


#endif

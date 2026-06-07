/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2)
 *
 * The mt* -> libnfs handlers. Each handler takes the server context (libnfs
 * handle + node table + port), an oid identifying the target object, and the
 * relevant message fields; it returns a negative errno on failure (Phoenix
 * VFS convention) or a non-negative result.
 *
 * Single-threaded: every handler runs on the one msgRecv loop thread, so the
 * libnfs sync context needs no locking. (MT is plan §8, deferred.)
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#ifndef _NFS_OPS_H_
#define _NFS_OPS_H_

#include <stdint.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/msg.h>

#include "nfs_node.h"

struct nfs_context;


typedef struct {
	struct nfs_context *nfs; /* the libnfs sync context */
	nfs_nodeTree_t nodes;    /* id<->path table */
	uint32_t port;           /* this server's port */
	oid_t parent;            /* parent fs oid for ".." at the mount root */
} nfs_fs_t;


extern int nfs_ops_lookup(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *res, oid_t *dev);
extern int nfs_ops_open(nfs_fs_t *fs, oid_t *oid);
extern int nfs_ops_close(nfs_fs_t *fs, oid_t *oid);
extern int nfs_ops_read(nfs_fs_t *fs, oid_t *oid, off_t offs, void *buf, size_t len);
extern int nfs_ops_write(nfs_fs_t *fs, oid_t *oid, off_t offs, const void *buf, size_t len);
extern int nfs_ops_truncate(nfs_fs_t *fs, oid_t *oid, size_t size);
extern int nfs_ops_getattr(nfs_fs_t *fs, oid_t *oid, int type, long long *attr);
extern int nfs_ops_getattrAll(nfs_fs_t *fs, oid_t *oid, struct _attrAll *attrs);
extern int nfs_ops_setattr(nfs_fs_t *fs, oid_t *oid, int type, long long val, const void *data, size_t size);
extern int nfs_ops_create(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *res, unsigned mode, int type, oid_t *dev);
extern int nfs_ops_destroy(nfs_fs_t *fs, oid_t *oid);
extern int nfs_ops_unlink(nfs_fs_t *fs, oid_t *dir, const char *name);
extern int nfs_ops_link(nfs_fs_t *fs, oid_t *dir, const char *name, oid_t *oid);
extern int nfs_ops_readdir(nfs_fs_t *fs, oid_t *dir, off_t offs, struct dirent *dent, size_t size);
extern int nfs_ops_statfs(nfs_fs_t *fs, void *buf, size_t len);


#endif

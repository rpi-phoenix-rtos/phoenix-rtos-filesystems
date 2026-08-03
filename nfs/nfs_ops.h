/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2)
 *
 * The mt* -> libnfs handlers. Each handler takes the server context (libnfs
 * handle + node table + port), an oid identifying the target object, and the
 * relevant message fields; it returns a negative errno on failure (Phoenix
 * VFS convention) or a non-negative result.
 *
 * Single-threaded: every handler runs on the one msgRecv loop thread, so the
 * libnfs sync context needs no locking. (MT is plan §8, deferred.) The periodic
 * NFSv4 lease keepalive is NOT an exception to this: the renew helper thread does
 * not touch libnfs; it self-sends an NFS_MSG_RENEW message so the renew executes
 * on the same loop thread (see srv.c nfs_renewThread / nfs_ops_renew).
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
	/* Mount parameters, captured once at mount time, so the loop thread can
	 * rebuild the libnfs context (re-running SETCLIENTID) after an NFSv4
	 * lease/state expiry — see nfs_ops_renew / the reclaim path in nfs_ops.c.
	 * server/export point into argv (stable for the process lifetime). */
	const char *server;
	const char *export;
	int version;
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

/* Keep the NFSv4 lease alive (send a RENEW) and, if it has already lapsed,
 * re-establish client state. Called on the loop thread in response to the
 * self-sent NFS_MSG_RENEW tick. Returns 0 on success, -errno otherwise (the
 * caller treats a failure as non-fatal; the reclaim path is the safety net). */
extern int nfs_ops_renew(nfs_fs_t *fs);

/* (Re-)create a libnfs context with this server's fixed transfer parameters.
 * Defined in srv.c (single source of truth for the tuning); declared here so
 * the reclaim path in nfs_ops.c can rebuild the context identically. */
extern struct nfs_context *nfs_makeContext(int version);


#endif

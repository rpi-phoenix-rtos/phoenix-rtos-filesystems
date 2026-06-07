/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2)
 *
 * Node table: maps the Phoenix handle space (oid.id) onto libnfs's
 * path-based API. Each node caches an export-relative absolute path; opened
 * files additionally cache the libnfs `struct nfsfh *`. The export root is
 * always id 0 (mirrors dummyfs DUMMYFS_ROOTID).
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#ifndef _NFS_NODE_H_
#define _NFS_NODE_H_

#include <stdint.h>
#include <sys/types.h>
#include <sys/rb.h>
#include <sys/file.h> /* otDir/otFile/otSymlink/otDev/otUnknown + atX attr enums */

struct nfsfh;

#define NFS_ROOTID 0


typedef struct nfs_node {
	rbnode_t idLinkage;   /* keyed by id (id -> node) */
	rbnode_t pathLinkage; /* keyed by path string (path -> node) */
	id_t id;              /* the oid.id handed back to the VFS */
	char *path;           /* canonical export-relative absolute path ("/" = root) */
	struct nfsfh *fh;     /* non-NULL while open; else NULL */
	int type;             /* otDir/otFile/otSymlink/otDev (from nfs_lstat64, don't-follow) */
	unsigned refs;        /* mtOpen refcount */
} nfs_node_t;


typedef struct nfs_nodeTree {
	rbtree_t byId;
	rbtree_t byPath;
	id_t nextId;
} nfs_nodeTree_t;


/* Initialize the table and create the root node (id 0, path "/"). */
extern int nfs_node_init(nfs_nodeTree_t *t);

/* Find a node by id. Returns NULL if absent. */
extern nfs_node_t *nfs_node_find(nfs_nodeTree_t *t, id_t id);

/* Find a node by path WITHOUT allocating. Returns NULL if absent. */
extern nfs_node_t *nfs_node_findPath(nfs_nodeTree_t *t, const char *path);

/* Find by path, or allocate+insert a new node for it (idempotent: repeated
 * lookups of the same path return the same node/id). Returns NULL on OOM. */
extern nfs_node_t *nfs_node_get(nfs_nodeTree_t *t, const char *path);

/* Remove and free a node (used on destroy/unlink-last-close). */
extern void nfs_node_remove(nfs_nodeTree_t *t, nfs_node_t *n);

/* Join a parent directory path and a single name component into a freshly
 * malloc'd canonical export-relative path. Caller frees. Returns NULL on OOM. */
extern char *nfs_node_joinPath(const char *parent, const char *name);


#endif

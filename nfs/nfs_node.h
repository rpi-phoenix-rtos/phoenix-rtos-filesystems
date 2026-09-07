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

#include <nfsc/libnfs.h> /* struct nfs_stat_64 (cached attributes, see below) */

struct nfsfh;

#define NFS_ROOTID 0


typedef struct nfs_node {
	rbnode_t idLinkage;   /* keyed by id (id -> node) */
	rbnode_t pathLinkage; /* keyed by path string (path -> node) */
	id_t id;              /* the oid.id handed back to the VFS */
	char *path;           /* canonical export-relative absolute path ("/" = root) */
	struct nfsfh *fh;     /* non-NULL while open; else NULL */
	int type;             /* otDir/otFile/otSymlink/otDev (from nfs_lstat64, don't-follow) */
	unsigned mode;        /* only for a spliced special file (mnt set, type otDev): the mode
	                         mkfifo()/mknod() asked for. Such a node has NO file on the export,
	                         so a stat has to be answered from here or not at all. */
	unsigned refs;        /* mtOpen refcount */
	oid_t mnt;            /* mounted-child oid spliced here via mtSetAttr(atDev); mnt.port==0
	                         means "no mount" (so lookup returns this node itself). Mirrors the
	                         dummyfs object `dev` field — required so a child fs (e.g. devfs at
	                         /dev once the NFS export owns "/") is reachable. (#153 T3 design-A) */
	/* Lazy-close fh cache (#156): when refs falls to 0 with fh != NULL, the node
	 * is parked on the tree's idle LRU instead of nfs_close()ing the fh, so the
	 * next open (e.g. the loader faulting the next page) reuses the open fh and
	 * skips the nfs_open/nfs_close RPC pair. Handle caching only — no data is
	 * cached, so no stale-data risk. idle != 0 iff the node is on idleHead. */
	int idle;                  /* 1 while parked on the idle LRU, else 0 */
	struct nfs_node *idleNext; /* idle LRU (MRU at head); valid only while idle != 0 */
	struct nfs_node *idlePrev;
	/* Positive attribute cache. Path resolution is quadratic without it: resolving a
	 * depth-d absolute path makes libphoenix issue one lookup per prefix, and each
	 * lookup lstats EVERY component of its prefix -- d(d+1)/2 round trips to describe
	 * d distinct paths (measured: 26 round trips at d=5, ~1.37 ms each). Remembering a
	 * component's don't-follow stat for a few milliseconds collapses that to one lstat
	 * per distinct path. Valid while attrValid != 0 and CLOCK_MONOTONIC has not passed
	 * it; every op that mutates the object -- or frees its name -- clears it, and
	 * mtOpen deliberately bypasses it (see nfs_ops.c). */
	struct nfs_stat_64 attr;
	uint64_t attrValid;        /* CLOCK_MONOTONIC deadline in us; 0 = nothing cached */
	/* Sequential-scan directory snapshot. POSIX readdir hands back ONE entry per
	 * call, and libnfs's nfs_opendir snapshots the whole directory (a READDIR round
	 * trip, or several for a big one). Re-opening per call therefore cost a full
	 * listing PER ENTRY: ~38 ms per readdir() on a 649-entry export directory, so
	 * walking one such directory took ~25 s and the cost grew with its size. Hold
	 * the snapshot open across the scan and step through it locally instead: one
	 * listing per scan. dirOffs is the cookie the snapshot is positioned at; a
	 * caller that seeks anywhere else (a rewind to 0 included) re-snapshots, which
	 * is also what keeps a re-scan from seeing a stale listing. At most one node
	 * holds a snapshot at a time (nfs_fs_t.scanNode). */
	struct nfsdir *dirCache;
	off_t dirOffs;
	int pathDetached;          /* 1 once unbound from byPath by an unlink-while-open (the file was
	                              removed but open fds keep it alive). The node stays reachable by id
	                              for those fds, but its name is free for a fresh node so a later
	                              create/lookup of the same path does not alias this (now orphaned)
	                              node's cached fh. */
} nfs_node_t;


typedef struct nfs_nodeTree {
	rbtree_t byId;
	rbtree_t byPath;
	id_t nextId;
	nfs_node_t *idleHead; /* MRU end of the lazy-close idle LRU (NULL = empty) */
	nfs_node_t *idleTail; /* LRU end; evicted first when idleCount > NFS_IDLE_MAX */
	unsigned idleCount;   /* number of nodes currently on the idle LRU */
} nfs_nodeTree_t;


/* Cap on cached-open fhs parked on the idle LRU. Bounds open-fh consumption on
 * the NFS server/client so lazy-close can't exhaust either; the tail is evicted
 * (nfs_close) once exceeded. Small: the loader pages one file at a time. */
#define NFS_IDLE_MAX 16


/* Initialize the table and create the root node (id 0, path "/"). */
extern int nfs_node_init(nfs_nodeTree_t *t);

/* Find a node by id. Returns NULL if absent. */
extern nfs_node_t *nfs_node_find(nfs_nodeTree_t *t, id_t id);

/* Find a node by path WITHOUT allocating. Returns NULL if absent. */
extern nfs_node_t *nfs_node_findPath(nfs_nodeTree_t *t, const char *path);

/* Find by path, or allocate+insert a new node for it (idempotent: repeated
 * lookups of the same path return the same node/id). Returns NULL on OOM. */
extern nfs_node_t *nfs_node_get(nfs_nodeTree_t *t, const char *path);

/* Remove and free a node (used on destroy/unlink-last-close). Defensively
 * unlinks the node from the idle LRU first; the caller is responsible for
 * nfs_close()ing n->fh (which needs the libnfs context) beforehand. */
extern void nfs_node_remove(nfs_nodeTree_t *t, nfs_node_t *n);

/* Unbind a still-open node from the byPath index (unlink-while-open). The node
 * stays in byId (open fds keep resolving it) but a later create/lookup of the
 * same path mints a fresh node instead of aliasing this orphaned one. Idempotent;
 * a no-op for the root. n->path/n->fh are intentionally left intact for the fds. */
extern void nfs_node_detachPath(nfs_nodeTree_t *t, nfs_node_t *n);

/* Lazy-close idle LRU (#156), structural only (no nfs_close — see nfs_ops.c):
 * push a node to the MRU head (sets idle, bumps idleCount) and unlink it
 * (clears idle, decrements idleCount). idle_unlink is a no-op if not parked. */
extern void nfs_node_idlePush(nfs_nodeTree_t *t, nfs_node_t *n);
extern void nfs_node_idleUnlink(nfs_nodeTree_t *t, nfs_node_t *n);

/* Return the LRU (tail) node for eviction, or NULL if the idle LRU is empty.
 * Does not unlink it — the caller nfs_close()s the fh then idleUnlinks. */
extern nfs_node_t *nfs_node_idleLru(nfs_nodeTree_t *t);

/* Drop every cached filehandle and reset the idle LRU, structurally (no
 * nfs_close). Used by the NFSv4 state-expiry reclaim (nfs_ops.c): after the
 * libnfs context is rebuilt, all previously cached fhs belong to the destroyed
 * context and their open stateids are dead, so they must be forgotten. The
 * id<->path table is preserved (paths are stable). Also drops every cached
 * attribute: the reclaim implies we lost track of what the server did meanwhile. */
extern void nfs_node_invalidateHandles(nfs_nodeTree_t *t);

/* Join a parent directory path and a single name component into a freshly
 * malloc'd canonical export-relative path. Caller frees. Returns NULL on OOM. */
extern char *nfs_node_joinPath(const char *parent, const char *name);


#endif

/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2)
 *
 * Node table implementation: two red-black trees over the same node set,
 * keyed by id and by path, plus a monotone id allocator.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/rb.h>

#include "nfs_node.h"


static int nfs_cmpId(rbnode_t *n1, rbnode_t *n2)
{
	nfs_node_t *a = lib_treeof(nfs_node_t, idLinkage, n1);
	nfs_node_t *b = lib_treeof(nfs_node_t, idLinkage, n2);

	if (a->id < b->id) {
		return -1;
	}
	if (a->id > b->id) {
		return 1;
	}
	return 0;
}


static int nfs_cmpPath(rbnode_t *n1, rbnode_t *n2)
{
	nfs_node_t *a = lib_treeof(nfs_node_t, pathLinkage, n1);
	nfs_node_t *b = lib_treeof(nfs_node_t, pathLinkage, n2);

	return strcmp(a->path, b->path);
}


int nfs_node_init(nfs_nodeTree_t *t)
{
	lib_rbInit(&t->byId, nfs_cmpId, NULL);
	lib_rbInit(&t->byPath, nfs_cmpPath, NULL);
	t->nextId = NFS_ROOTID + 1;
	t->idleHead = NULL;
	t->idleTail = NULL;
	t->idleCount = 0;

	nfs_node_t *root = calloc(1, sizeof(nfs_node_t));
	if (root == NULL) {
		return -ENOMEM;
	}

	root->id = NFS_ROOTID;
	root->path = strdup("/");
	if (root->path == NULL) {
		free(root);
		return -ENOMEM;
	}
	root->fh = NULL;
	root->type = otDir;
	root->refs = 0;
	root->mnt.port = 0; /* no child mounted here yet (calloc already zeroed it) */

	lib_rbInsert(&t->byId, &root->idLinkage);
	lib_rbInsert(&t->byPath, &root->pathLinkage);

	return 0;
}


nfs_node_t *nfs_node_find(nfs_nodeTree_t *t, id_t id)
{
	nfs_node_t key;
	key.id = id;

	rbnode_t *r = lib_rbFind(&t->byId, &key.idLinkage);
	if (r == NULL) {
		return NULL;
	}
	return lib_treeof(nfs_node_t, idLinkage, r);
}


nfs_node_t *nfs_node_findPath(nfs_nodeTree_t *t, const char *path)
{
	nfs_node_t key;
	key.path = (char *)path;

	rbnode_t *r = lib_rbFind(&t->byPath, &key.pathLinkage);
	if (r == NULL) {
		return NULL;
	}
	return lib_treeof(nfs_node_t, pathLinkage, r);
}


nfs_node_t *nfs_node_get(nfs_nodeTree_t *t, const char *path)
{
	nfs_node_t *n = nfs_node_findPath(t, path);
	if (n != NULL) {
		return n;
	}

	n = calloc(1, sizeof(nfs_node_t));
	if (n == NULL) {
		return NULL;
	}

	n->path = strdup(path);
	if (n->path == NULL) {
		free(n);
		return NULL;
	}

	n->id = t->nextId++;
	n->fh = NULL;
	n->type = otUnknown;
	n->refs = 0;

	lib_rbInsert(&t->byId, &n->idLinkage);
	lib_rbInsert(&t->byPath, &n->pathLinkage);

	return n;
}


void nfs_node_remove(nfs_nodeTree_t *t, nfs_node_t *n)
{
	if ((n == NULL) || (n->id == NFS_ROOTID)) {
		/* Never evict the root node. */
		return;
	}

	/* Defensive: a node about to be freed must not linger on the idle LRU
	 * (#156). The caller closed n->fh already; here we only fix the links. */
	nfs_node_idleUnlink(t, n);

	lib_rbRemove(&t->byId, &n->idLinkage);
	/* A node already unbound by nfs_node_detachPath() is no longer in byPath;
	 * removing it again would corrupt the tree. */
	if (n->pathDetached == 0) {
		lib_rbRemove(&t->byPath, &n->pathLinkage);
	}
	free(n->path);
	free(n);
}


void nfs_node_detachPath(nfs_nodeTree_t *t, nfs_node_t *n)
{
	if ((n == NULL) || (n->id == NFS_ROOTID) || (n->pathDetached != 0)) {
		return;
	}

	/* Defensive: a still-open node shouldn't be on the idle LRU (refs > 0), but
	 * keep the links consistent regardless. */
	nfs_node_idleUnlink(t, n);

	lib_rbRemove(&t->byPath, &n->pathLinkage);
	n->pathDetached = 1;
}


void nfs_node_idlePush(nfs_nodeTree_t *t, nfs_node_t *n)
{
	if ((n == NULL) || (n->idle != 0)) {
		return;
	}

	n->idlePrev = NULL;
	n->idleNext = t->idleHead;
	if (t->idleHead != NULL) {
		t->idleHead->idlePrev = n;
	}
	t->idleHead = n;
	if (t->idleTail == NULL) {
		t->idleTail = n;
	}
	n->idle = 1;
	t->idleCount++;
}


void nfs_node_idleUnlink(nfs_nodeTree_t *t, nfs_node_t *n)
{
	if ((n == NULL) || (n->idle == 0)) {
		return;
	}

	if (n->idlePrev != NULL) {
		n->idlePrev->idleNext = n->idleNext;
	}
	else {
		t->idleHead = n->idleNext;
	}
	if (n->idleNext != NULL) {
		n->idleNext->idlePrev = n->idlePrev;
	}
	else {
		t->idleTail = n->idlePrev;
	}

	n->idleNext = NULL;
	n->idlePrev = NULL;
	n->idle = 0;
	t->idleCount--;
}


nfs_node_t *nfs_node_idleLru(nfs_nodeTree_t *t)
{
	return t->idleTail;
}


void nfs_node_invalidateHandles(nfs_nodeTree_t *t)
{
	if (t->byId.root == NULL) {
		return;
	}

	/* Walk every node and drop its cached filehandle. The libnfs context that
	 * owned these fhs has been destroyed (nfs_reclaim), so the pointers dangle
	 * and their NFSv4 open stateids are dead; we must NOT nfs_close them (the
	 * owning context is gone). This is structural only — the id<->path table is
	 * left intact (paths are stable), so each subsequent op re-opens by path. */
	for (rbnode_t *it = lib_rbMinimum(t->byId.root); it != NULL; it = lib_rbNext(it)) {
		nfs_node_t *n = lib_treeof(nfs_node_t, idLinkage, it);
		n->fh = NULL;
		n->idle = 0;
		n->idleNext = NULL;
		n->idlePrev = NULL;
	}

	/* The idle LRU referenced the now-invalidated fhs; reset it wholesale. */
	t->idleHead = NULL;
	t->idleTail = NULL;
	t->idleCount = 0;
}


char *nfs_node_joinPath(const char *parent, const char *name)
{
	size_t plen = strlen(parent);
	size_t nlen = strlen(name);

	/* "/" + name, collapsing a trailing slash on parent. */
	char *out = malloc(plen + 1 + nlen + 1);
	if (out == NULL) {
		return NULL;
	}

	memcpy(out, parent, plen);
	if ((plen == 0) || (out[plen - 1] != '/')) {
		out[plen++] = '/';
	}
	memcpy(out + plen, name, nlen);
	out[plen + nlen] = '\0';

	return out;
}

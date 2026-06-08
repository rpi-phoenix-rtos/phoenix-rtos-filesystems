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

	lib_rbRemove(&t->byId, &n->idLinkage);
	lib_rbRemove(&t->byPath, &n->pathLinkage);
	free(n->path);
	free(n);
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

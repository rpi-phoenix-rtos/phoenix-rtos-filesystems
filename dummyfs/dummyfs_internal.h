/*
 * Phoenix-RTOS
 *
 * dummyfs
 *
 * Copyright 2012, 2018, 2023 Phoenix Systems
 * Copyright 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Kamil Amanowicz,
 * Maciej Purski, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef DUMMYFS_INTERNAL_H_
#define DUMMYFS_INTERNAL_H_

#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <sys/file.h>
#include <sys/rb.h>
#include <posix/idtree.h>

#include <board_config.h>

#ifndef DUMMYFS_SIZE_MAX
#define DUMMYFS_SIZE_MAX (32 * 1024 * 1024)
#endif

#ifndef DUMMYFS_NAME_MAX
#define DUMMYFS_NAME_MAX 255U
#endif

#if 0
#define TRACE() printf("%s\n", __FUNCTION__)
#else
#define TRACE()
#endif


typedef struct _dummyfs_dirent_t {
	rbnode_t linkage;
	uint32_t key;
	struct _dummyfs_dirent_t *prev; /* entries with the same key */
	struct _dummyfs_dirent_t *next;

	off_t pos;                       /* readdir position, unique in the directory and never reused */
	struct _dummyfs_dirent_t *lprev; /* all entries, in position order */
	struct _dummyfs_dirent_t *lnext;

	char *name;
	size_t len;
	uint32_t type;
	oid_t oid;
} dummyfs_dirent_t;


typedef struct {
	oid_t oid;
	oid_t dev;

	unsigned int uid;
	unsigned int gid;
	uint32_t mode;

	int refs;
	int nlink;

	idnode_t node;
	size_t size;

	union {
		struct {
			rbtree_t tree;
			size_t entries;
			dummyfs_dirent_t *list; /* entries in position order */
			off_t nextPos;          /* position of the next entry added */
			struct {
				off_t offs;
				dummyfs_dirent_t *entry; /* first entry at or after offs, NULL if unknown */
			} hint;    /* Hint for ls iteration */
		} dir;         /* Used for directories */
		void *data;    /* Used for small files */
		void **chunks; /* Used for big files */
	};

	time_t atime;
	time_t mtime;
	time_t ctime;
} dummyfs_object_t;


typedef struct {
	uint32_t port;
	handle_t mutex;
	size_t size;
	idtree_t dummytree;
	char *mountpt;
	oid_t parent;
	unsigned long mode;
} dummyfs_t;


int _dummyfs_destroy(dummyfs_t *fs, oid_t *oid);


#endif /* DUMMYFS_INTERNAL_H_ */

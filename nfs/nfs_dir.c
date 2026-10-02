/*
 * Phoenix-RTOS — NFS filesystem server
 *
 * Directory listing snapshot with readdir positions that survive removals.
 * See nfs_dir.h.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "nfs_dir.h"


off_t nfs_dir_pos(const char *name, size_t len)
{
	/* 64-bit FNV-1a */
	uint64_t h = 0xcbf29ce484222325ULL;

	for (size_t i = 0; i < len; i++) {
		h ^= (uint8_t)name[i];
		h *= 0x100000001b3ULL;
	}

	/* 62 bits: the largest position, plus one for the position after it,
	 * stays below INT64_MAX */
	return (off_t)(h >> 2) + NFS_DIR_FIRST;
}


nfs_dirSnap_t *nfs_dir_new(void)
{
	return calloc(1, sizeof(nfs_dirSnap_t));
}


void nfs_dir_free(nfs_dirSnap_t *s)
{
	if (s == NULL) {
		return;
	}

	for (size_t i = 0; i < s->n; i++) {
		free(s->ents[i].name);
	}
	free(s->ents);
	free(s);
}


int nfs_dir_add(nfs_dirSnap_t *s, const char *name, uint64_t ino, int type)
{
	if (s->n == s->cap) {
		size_t cap = (s->cap == 0) ? 64 : (2 * s->cap);
		nfs_dirEntry_t *ents = realloc(s->ents, cap * sizeof(*ents));
		if (ents == NULL) {
			return -ENOMEM;
		}
		s->ents = ents;
		s->cap = cap;
	}

	nfs_dirEntry_t *e = &s->ents[s->n];
	e->namelen = strlen(name);
	e->name = malloc(e->namelen + 1);
	if (e->name == NULL) {
		return -ENOMEM;
	}
	memcpy(e->name, name, e->namelen + 1);
	e->pos = nfs_dir_pos(name, e->namelen);
	e->lpos = 0;
	e->ino = ino;
	e->type = type;
	e->gone = 0;
	s->n++;

	return 0;
}


static int nfs_dir_cmp(const void *a, const void *b)
{
	const nfs_dirEntry_t *ea = a;
	const nfs_dirEntry_t *eb = b;

	if (ea->pos != eb->pos) {
		return (ea->pos < eb->pos) ? -1 : 1;
	}

	return strcmp(ea->name, eb->name);
}


void nfs_dir_sort(nfs_dirSnap_t *s)
{
	if (s->n > 1) {
		qsort(s->ents, s->n, sizeof(s->ents[0]), nfs_dir_cmp);
	}

	off_t lpos = NFS_DIR_FIRST;
	for (size_t i = 0; i < s->n; i++) {
		s->ents[i].lpos = lpos;
		lpos += (off_t)s->ents[i].namelen;
	}
}


/* Index of the first entry whose key is >= offs (s->n if none) */
static size_t nfs_dir_lowerBound(const nfs_dirSnap_t *s, off_t offs, int legacy)
{
	size_t lo = 0, hi = s->n;

	while (lo < hi) {
		size_t mid = lo + (hi - lo) / 2;
		off_t key = (legacy != 0) ? s->ents[mid].lpos : s->ents[mid].pos;
		if (key < offs) {
			lo = mid + 1;
		}
		else {
			hi = mid;
		}
	}

	return lo;
}


void nfs_dir_remove(nfs_dirSnap_t *s, const char *name)
{
	if (s == NULL) {
		return;
	}

	size_t len = strlen(name);
	off_t pos = nfs_dir_pos(name, len);

	for (size_t i = nfs_dir_lowerBound(s, pos, 0); (i < s->n) && (s->ents[i].pos == pos); i++) {
		if ((s->ents[i].namelen == len) && (memcmp(s->ents[i].name, name, len) == 0)) {
			s->ents[i].gone = 1;
			break;
		}
	}
}


const nfs_dirEntry_t *nfs_dir_seek(const nfs_dirSnap_t *s, off_t offs)
{
	for (size_t i = nfs_dir_lowerBound(s, offs, 0); i < s->n; i++) {
		if (s->ents[i].gone == 0) {
			return &s->ents[i];
		}
	}

	return NULL;
}


const nfs_dirEntry_t *nfs_dir_seekLegacy(const nfs_dirSnap_t *s, off_t offs, off_t *end)
{
	for (size_t i = nfs_dir_lowerBound(s, offs, 1); i < s->n; i++) {
		if (s->ents[i].gone == 0) {
			*end = s->ents[i].lpos + (off_t)s->ents[i].namelen;
			return &s->ents[i];
		}
	}

	return NULL;
}


const nfs_dirEntry_t *nfs_dir_read(const nfs_dirSnap_t *s, off_t offs, int stable, off_t *end)
{
	if (stable == 0) {
		return nfs_dir_seekLegacy(s, offs, end);
	}

	const nfs_dirEntry_t *e = nfs_dir_seek(s, offs);
	if (e != NULL) {
		*end = e->pos + 1;
	}

	return e;
}

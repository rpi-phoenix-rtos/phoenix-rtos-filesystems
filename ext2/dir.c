/*
 * Phoenix-RTOS
 *
 * EXT2 filesystem
 *
 * Directory operations
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <sys/stat.h>

#include "block.h"
#include "dir.h"
#include "file.h"


/* Is this record well formed? A bad one would make a walk loop or overrun the block. */
static int _ext2_dir_valid(ext2_t *fs, const ext2_dirent_t *entry, uint32_t offs)
{
	return (entry->size >= sizeof(ext2_dirent_t)) && (entry->size <= fs->blocksz - offs) &&
		(sizeof(ext2_dirent_t) + entry->len <= entry->size);
}


static int _ext2_dir_isdot(const ext2_dirent_t *entry)
{
	return ((entry->len == 1) && (entry->name[0] == '.')) ||
		((entry->len == 2) && (entry->name[0] == '.') && (entry->name[1] == '.'));
}


/* Every block is walked: a directory that once spanned several blocks keeps
 * its emptied middle blocks (see _ext2_dir_remove()), so its size says nothing
 * about whether it is empty. */
int _ext2_dir_empty(ext2_t *fs, ext2_obj_t *dir)
{
	ext2_dirent_t *entry;
	uint32_t boffs, offs;
	ssize_t ret;
	char *buff;
	int empty = 1;

	if ((buff = (char *)malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	for (boffs = 0; (boffs < dir->inode->size) && (empty == 1); boffs += fs->blocksz) {
		if ((ret = _ext2_file_read(fs, dir, boffs, buff, fs->blocksz)) != fs->blocksz) {
			empty = (ret < 0) ? (int)ret : -EINVAL;
			break;
		}

		for (offs = 0; offs < fs->blocksz; offs += entry->size) {
			entry = (ext2_dirent_t *)(buff + offs);

			if (!_ext2_dir_valid(fs, entry, offs)) {
				empty = -EIO;
				break;
			}

			if ((entry->ino != 0) && !_ext2_dir_isdot(entry)) {
				empty = 0;
				break;
			}
		}
	}

	free(buff);

	return empty;
}


static int _ext2_dir_find(ext2_t *fs, ext2_obj_t *dir, const char *name, size_t len, char *buff, uint32_t *offs)
{
	ext2_dirent_t *entry;
	uint32_t boffs;
	ssize_t ret;

	for (boffs = *offs; boffs < dir->inode->size; boffs += fs->blocksz) {
		if ((ret = _ext2_file_read(fs, dir, boffs, buff, fs->blocksz)) != fs->blocksz)
			return (ret < 0) ? (int)ret : -EINVAL;

		for (*offs = 0; *offs < fs->blocksz; *offs += entry->size) {
			entry = (ext2_dirent_t *)(buff + *offs);

			if (!entry->size)
				break;

			/* inode 0: an unused record whose old name may still be there */
			if ((entry->ino != 0) && ((size_t)entry->len == len) && !strncmp(entry->name, name, len))
				return boffs;
		}
	}

	return -ENOENT;
}


int _ext2_dir_search(ext2_t *fs, ext2_obj_t *dir, const char *name, size_t len, id_t *res)
{
	uint32_t offs = 0;
	char *buff;
	int err;

	if ((buff = (char *)malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	do {
		if ((err = _ext2_dir_find(fs, dir, name, len, buff, &offs)) < 0)
			break;

		*res = ((ext2_dirent_t *)(buff + offs))->ino;
	} while (0);

	free(buff);

	return err;
}


/* A position is the byte offset of an entry, so it stays valid while other
 * entries are removed: no removal moves a live entry (see _ext2_dir_remove()).
 * A position can still fall inside a record, when the entry it pointed at was
 * removed and merged into the previous one, so the block is walked from its
 * start to the first live entry at or after the position. */
int _ext2_dir_read(ext2_t *fs, ext2_obj_t *dir, off_t offs, struct dirent *res, size_t len, off_t *next)
{
	ext2_dirent_t *entry = NULL;
	uint32_t boffs, eoffs = 0;
	ssize_t ret;
	char *buff;
	int err = -ENOENT;

	if (offs < 0)
		return -EINVAL;

	if (!dir->inode->size || !dir->inode->links || (offs >= dir->inode->size))
		return -ENOENT;

	if ((buff = (char *)malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	for (boffs = (uint32_t)offs - (uint32_t)offs % fs->blocksz; boffs < dir->inode->size; boffs += fs->blocksz) {
		if ((ret = _ext2_file_read(fs, dir, boffs, buff, fs->blocksz)) != fs->blocksz) {
			err = (ret < 0) ? (int)ret : -EINVAL;
			break;
		}

		for (eoffs = 0; eoffs < fs->blocksz; eoffs += entry->size) {
			entry = (ext2_dirent_t *)(buff + eoffs);

			if (!_ext2_dir_valid(fs, entry, eoffs)) {
				err = -EIO;
				break;
			}

			if ((boffs + eoffs >= offs) && (entry->ino != 0) && (entry->len != 0)) {
				err = EOK;
				break;
			}
		}

		if (err != -ENOENT)
			break;
	}

	if (err < 0) {
		free(buff);
		return err;
	}

	if (len < sizeof(struct dirent) + entry->len + 1) {
		free(buff);
		return -EINVAL;
	}

	switch (entry->type) {
		case DIRENT_DIR:
			res->d_type = DT_DIR;
			break;

		case DIRENT_CHRDEV:
			res->d_type = DT_CHR;
			break;

		case DIRENT_BLKDEV:
			res->d_type = DT_BLK;
			break;

		default:
			res->d_type = DT_REG;
			break;
	}

	/* The next position is the end of this record. A client that advances by
	 * d_reclen instead of taking *next lands there too, unless the record is
	 * further than 64 KiB past the position asked for (a long run of emptied
	 * blocks), where d_reclen cannot reach and it reads this entry again. */
	off_t end = (off_t)boffs + eoffs + entry->size;
	res->d_ino = entry->ino;
	res->d_reclen = (uint16_t)(((end - offs) > UINT16_MAX) ? UINT16_MAX : (end - offs));
	res->d_namlen = entry->len;
	memcpy(res->d_name, entry->name, entry->len);
	res->d_name[entry->len] = '\0';
	free(buff);

	if (next != NULL)
		*next = end;

	dir->inode->atime = time(NULL);

	return res->d_reclen;
}


/* The directory's mtime and ctime (POSIX: updated when an entry is added or
 * removed) are set by the _ext2_file_write() or _ext2_file_truncate() that
 * stores the changed entry block, and the inode is synced before returning. */
int _ext2_dir_add(ext2_t *fs, ext2_obj_t *dir, const char *name, size_t len, uint16_t mode, uint32_t ino)
{
	uint32_t offs, size = 0;
	ext2_dirent_t *entry;
	char *buff;
	ssize_t ret;

	if (len > MAX_NAMELEN) {
		return -ENAMETOOLONG;
	}

	if ((buff = (char *)malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	if (!dir->inode->size) {
		offs = fs->blocksz;
	}
	else {
		if ((ret = _ext2_file_read(fs, dir, dir->inode->size - fs->blocksz, buff, fs->blocksz)) != fs->blocksz) {
			free(buff);
			return (ret < 0) ? (int)ret : -EINVAL;
		}

		for (offs = 0; offs < fs->blocksz; offs += entry->size) {
			entry = (ext2_dirent_t *)(buff + offs);

			if (!entry->size)
				break;

			if (offs + entry->size == fs->blocksz) {
				entry->size = (entry->len) ? (entry->len + sizeof(ext2_dirent_t) + 3) & ~3 : 0;
				offs += entry->size;

				size = (len + sizeof(ext2_dirent_t) + 3) & ~3;
				if (size >= fs->blocksz - offs) {
					entry->size += fs->blocksz - offs;
					offs = fs->blocksz;
					break;
				}

				size = fs->blocksz - offs;
				break;
			}
		}
	}

	/* No space in this block => alloc new one */
	if (offs >= fs->blocksz) {
		dir->inode->size += fs->blocksz;
		memset(buff, 0, fs->blocksz);
		size = fs->blocksz;
		offs = 0;
	}

	entry = (ext2_dirent_t *)(buff + offs);
	entry->ino = ino;
	entry->size = size;
	entry->len = len;
	memcpy(entry->name, name, len);

	if (S_ISDIR(mode))
		entry->type = DIRENT_DIR;
	else if (S_ISCHR(mode))
		entry->type = DIRENT_CHRDEV;
	else if (S_ISBLK(mode))
		entry->type = DIRENT_BLKDEV;
	else if (S_ISFIFO(mode))
		entry->type = DIRENT_FIFO;
	else if (S_ISSOCK(mode))
		entry->type = DIRENT_SOCK;
	else if (S_ISREG(mode))
		entry->type = DIRENT_FILE;
	else
		entry->type = DIRENT_UNKNOWN;

	offs = (dir->inode->size > fs->blocksz) ? dir->inode->size - fs->blocksz : 0;

	if ((ret = _ext2_file_write(fs, dir, offs, buff, fs->blocksz)) != fs->blocksz) {
		free(buff);
		return (ret < 0) ? (int)ret : -EINVAL;
	}

	free(buff);

	return EOK;
}


int _ext2_dir_remove(ext2_t *fs, ext2_obj_t *dir, const char *name, size_t len)
{
	ext2_dirent_t *entry, *tmp;
	uint32_t prev, boffs, size, offs = 0;
	ssize_t ret;
	char *buff;
	int err;

	if ((buff = (char *)malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	if ((err = _ext2_dir_find(fs, dir, name, len, buff, &offs)) < 0) {
		free(buff);
		return err;
	}

	entry = (ext2_dirent_t *)(buff + offs);
	boffs = err;

	/* No live entry moves here: positions handed out by _ext2_dir_read() are
	 * entry offsets, and a scan that removes what it reads (rm -rf) must find
	 * the remaining entries where it left them. */

	/* Entry in the middle of the block => expand previous entry size */
	if (offs) {
		for (prev = 0, tmp = (ext2_dirent_t *)buff; prev + tmp->size < offs;) {
			prev += tmp->size;
			tmp = (ext2_dirent_t *)(buff + prev);
		}
		tmp->size += entry->size;

		if ((ret = _ext2_file_write(fs, dir, boffs, buff, fs->blocksz)) != fs->blocksz)
			err = (ret < 0) ? (int)ret : -EINVAL;
		else
			err = EOK;
	}
	/* Entry takes the entire last block => truncate it, with any emptied blocks before it */
	else if ((entry->size == fs->blocksz) && (boffs + fs->blocksz >= dir->inode->size)) {
		size = dir->inode->size - fs->blocksz;

		/* Block 0 holds "." and "..", so it is never empty. A block that cannot
		 * be read is kept: it only costs space. */
		while (size > fs->blocksz) {
			if (_ext2_file_read(fs, dir, size - fs->blocksz, buff, fs->blocksz) != fs->blocksz)
				break;

			tmp = (ext2_dirent_t *)buff;
			if ((tmp->ino != 0) || (tmp->size != fs->blocksz))
				break;

			size -= fs->blocksz;
		}

		err = _ext2_file_truncate(fs, dir, size);

		/* _ext2_file_truncate() updates the directory's size, blocks, mtime and
		 * ctime in memory only, unlike _ext2_file_write() in the other branches,
		 * which syncs the inode. Without this the on-disk directory kept its old
		 * size and timestamps (and still referenced the freed block) until the
		 * object was evicted or the filesystem unmounted. */
		if (err == EOK)
			err = _ext2_obj_sync(fs, dir);
	}
	/* Entry at the start of a block => keep its record as an unused one (inode 0) */
	else {
		entry->ino = 0;

		if ((ret = _ext2_file_write(fs, dir, boffs, buff, fs->blocksz)) != fs->blocksz)
			err = (ret < 0) ? (int)ret : -EINVAL;
		else
			err = EOK;
	}

	free(buff);

	return err;
}

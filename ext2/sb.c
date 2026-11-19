/*
 * Phoenix-RTOS
 *
 * EXT2 filesystem
 *
 * SuperBlock
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "sb.h"


/* Writes the superblock out.
 *
 * The only superblock fields this driver changes are the filesystem-wide free
 * block and inode counts, and those are a summary: each is the sum of the
 * per-group counts in the group descriptors, which are written together with
 * the bitmaps on every allocation and release. So the superblock is written
 * only on close, sync and unmount, not after every operation. Writing it after
 * every operation made it one of the six device writes each small write() cost
 * (the others are the data, the inode, the indirect block, the block bitmap
 * and its group descriptor), without making anything safer.
 *
 * After a power cut the on-disk counts may therefore be behind. That is not an
 * inconsistency anyone acts on: ext2_gdt_init() recomputes them from the
 * descriptors at mount, as fsck does, and fsck reports stale superblock counts
 * without failing the check.
 *
 * Nothing is written when the superblock has not changed, so closing a file
 * that was only read costs nothing. */
int ext2_sb_sync(ext2_t *fs)
{
	off_t offset = SB_OFFSET;
	void *data = fs->sb;
	size_t size = sizeof(ext2_sb_t);

	if (memcmp(fs->sb, fs->sbDisk, size) == 0) {
		return EOK;
	}

	if (fs->strg != NULL) {
		if (fs->strg->dev->blk->ops->write(fs->strg, fs->strg->start + offset, data, size) != size) {
			return -EIO;
		}
	}
	else if (fs->legacy.write != NULL) {
		if (fs->legacy.write(fs->legacy.devId, offset, data, size) != size) {
			return -EIO;
		}
	}
	else {
		return -ENOSYS;
	}

	memcpy(fs->sbDisk, fs->sb, size);

	return EOK;
}


void ext2_sb_destroy(ext2_t *fs)
{
	ext2_sb_sync(fs);
	free(fs->sb);
	free(fs->sbDisk);
}


int ext2_sb_init(ext2_t *fs)
{
	off_t offset = SB_OFFSET;
	size_t size = sizeof(ext2_sb_t);

	if ((fs->sb = (ext2_sb_t *)malloc(sizeof(ext2_sb_t))) == NULL)
		return -ENOMEM;

	if (fs->strg != NULL) {
		if (fs->strg->dev->blk->ops->read(fs->strg, fs->strg->start + offset, fs->sb, size) != size) {
			free(fs->sb);
			return -EIO;
		}
	}
	else if (fs->legacy.read != NULL) {
		if (fs->legacy.read(fs->legacy.devId, offset, (char *)fs->sb, size) != size) {
			free(fs->sb);
			return -EIO;
		}
	}
	else {
		return -ENOSYS;
	}

	if (fs->sb->magic != MAGIC_EXT2) {
		free(fs->sb);
		return -ENOENT;
	}

	if ((fs->sbDisk = (ext2_sb_t *)malloc(sizeof(ext2_sb_t))) == NULL) {
		free(fs->sb);
		return -ENOMEM;
	}

	memcpy(fs->sbDisk, fs->sb, size);

	//TODO: features check

	if (!fs->sb->inodesz)
		fs->sb->inodesz = 128;

	fs->blocksz = 1024 << fs->sb->logBlocksz;

	return EOK;
}

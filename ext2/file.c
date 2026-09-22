/*
 * Phoenix-RTOS
 *
 * EXT2 filesystem
 *
 * File operations
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/minmax.h>
#include <sys/stat.h>

#include "block.h"
#include "file.h"

/* Upper bound on a single coalesced read handed to the storage layer. Matches
 * the 64 KiB block-cache line, so a longer run cannot avoid another fetch. */
#define EXT2_READ_RUN_MAX (64u * 1024u)


ssize_t _ext2_file_read(ext2_t *fs, ext2_obj_t *obj, off_t offs, char *buff, size_t len)
{
	uint32_t block = offs / fs->blocksz;
	uint32_t blkEnd;
	/* Longest run handed to the storage layer in one call, in blocks. */
	const uint32_t runMax = (EXT2_READ_RUN_MAX / fs->blocksz) > 0u ? (EXT2_READ_RUN_MAX / fs->blocksz) : 1u;
	size_t l = 0;
	void *data;
	int err;

	if ((offs < 0) || (offs >= obj->inode->size))
		return 0;

	if (len > obj->inode->size - offs)
		len = obj->inode->size - offs;

	if (!len)
		return 0;

	if (S_ISLNK(obj->inode->mode) && (obj->inode->size <= MAX_SYMLINK_LEN_IN_INODE)) {
		memcpy((void *)buff, (const void *)(((const char *)obj->inode->block) + offs), len);
		return len;
	}

	if (offs % fs->blocksz || len < fs->blocksz) {
		if ((data = malloc(fs->blocksz)) == NULL)
			return -ENOMEM;

		if ((err = ext2_block_init(fs, obj, block, data)) < 0) {
			free(data);
			return err;
		}

		if ((l = fs->blocksz - offs % fs->blocksz) > len)
			l = len;

		memcpy(buff, data + offs % fs->blocksz, l);
		free(data);
		block++;
	}

	/* Read CONTIGUOUS physical runs in one call instead of one call per block.
	 * ext2_block_sync() has always coalesced runs on the write side; the read
	 * side did not, so a sequential read paid an ext2_block_get() plus a
	 * separate single-block ext2_block_read() for every 4 KiB. That per-block
	 * cost does not overlap the device transfer: a 64 KiB cache line fetched at
	 * the raw 59.1 MB/s takes 1.109 ms and serves 16 blocks, i.e. 69.3 us of
	 * device time per block, against 96.2 us observed -- and the 26.8 us
	 * difference is exactly the 42.6-vs-59.1 MB/s gap. */
	blkEnd = (offs + len) / fs->blocksz;
	while (block < blkEnd) {
		uint32_t *bno, first, n;

		if ((err = ext2_block_get(fs, obj, block, &bno)) < 0)
			return err;

		/* A hole reads as zeros; it has no physical run to coalesce. */
		if (*bno == 0) {
			memset(buff + l, 0, fs->blocksz);
			block++;
			l += fs->blocksz;
			continue;
		}

		/* Take the value now: a later ext2_block_get() may reload the indirect
		 * block this pointer refers into. */
		first = *bno;

		/* Cap the run. Before coalescing, this path never asked the storage
		 * layer for more than ONE block, so an unbounded run hands a driver a
		 * transfer far larger than anything it has ever seen -- and at 1 KiB
		 * blocks a contiguous 16 MB file is 16384 of them. That faulted the
		 * bcm2711-emmc driver serving the SD root. EXT2_READ_RUN_MAX bytes is
		 * the whole benefit anyway: it matches the block cache's line, so a
		 * longer run cannot save another fetch. */
		for (n = 1; ((block + n) < blkEnd) && ((n + 1u) <= runMax); n++) {
			if ((err = ext2_block_get(fs, obj, block + n, &bno)) < 0)
				return err;

			if (*bno != (first + n))
				break;
		}

		if ((err = ext2_block_read(fs, first, buff + l, n)) < 0)
			return err;

		block += n;
		l += (size_t)n * fs->blocksz;
	}

	if (len > l) {
		if ((data = malloc(fs->blocksz)) == NULL)
			return -ENOMEM;

		if ((err = ext2_block_init(fs, obj, block, data)) < 0) {
			free(data);
			return err;
		}

		memcpy(buff + l, data, len - l);
		free(data);
	}

	obj->inode->atime = time(NULL);

	return len;
}


ssize_t _ext2_file_write(ext2_t *fs, ext2_obj_t *obj, off_t offs, const char *buff, size_t len)
{
	if (len == 0) {
		return 0;
	}

	/* Link can only be written to during creation. */
	if (S_ISLNK(obj->inode->mode) && ((offs != 0) || (obj->inode->size != 0))) {
		return -EINVAL;
	}

	int err;
	if (S_ISLNK(obj->inode->mode) && (len <= MAX_SYMLINK_LEN_IN_INODE)) {
		memcpy((void *)(obj->inode->block), (const void *)buff, len);
	}
	else {
		uint32_t block = offs / fs->blocksz;
		uint32_t offsInBlock = offs % fs->blocksz;
		size_t written = 0;

		if ((offsInBlock != 0) || (len < fs->blocksz)) {
			void *data = malloc(fs->blocksz);
			if (data == NULL) {
				return -ENOMEM;
			}

			err = ext2_block_init(fs, obj, block, data);
			if (err < 0) {
				free(data);
				return err;
			}

			written = min(fs->blocksz - offsInBlock, len);
			memcpy(data + offsInBlock, buff, written);

			err = ext2_block_syncone(fs, obj, block, data);
			if (err < 0) {
				free(data);
				return err;
			}

			free(data);
			block++;
		}

		const uint32_t fullBlocks = (len - written) / fs->blocksz;
		if (fullBlocks > 0) {
			err = ext2_block_sync(fs, obj, block, buff + written, fullBlocks);
			if (err < 0) {
				return err;
			}

			written += fs->blocksz * fullBlocks;
			block += fullBlocks;
		}

		if (len > written) {
			void *data = malloc(fs->blocksz);
			if (data == NULL) {
				return -ENOMEM;
			}

			err = ext2_block_init(fs, obj, block, data);
			if (err < 0) {
				free(data);
				return err;
			}

			memcpy(data, buff + written, len - written);

			err = ext2_block_syncone(fs, obj, block, data);
			if (err < 0) {
				free(data);
				return err;
			}

			free(data);
		}
	}

	if ((offs + len) > obj->inode->size) {
		obj->inode->size = offs + len;
	}

	obj->inode->mtime = obj->inode->ctime = time(NULL);
	obj->flags |= OFLAG_DIRTY;

	err = _ext2_obj_sync(fs, obj);
	if (err < 0) {
		return err;
	}

	err = ext2_sb_sync(fs);
	if (err < 0) {
		return err;
	}

	return len;
}


int _ext2_file_truncate(ext2_t *fs, ext2_obj_t *obj, size_t size)
{
	uint32_t *bno, block, first = 0, run = 0, freed = 0;
	uint32_t start = (size + fs->blocksz - 1) / fs->blocksz;
	uint32_t end = (obj->inode->size + fs->blocksz - 1) / fs->blocksz;
	int err;

	/* A short symlink keeps its target INSIDE the block array, and a device
	 * node keeps its rdev there, so those uint32_t are not block numbers.
	 * Walking them would hand arbitrary values to ext2_block_destroy() and
	 * free blocks belonging to other files. Neither owns storage to release. */
	if (EXT2_ISDEV(obj->inode->mode) ||
			(S_ISLNK(obj->inode->mode) && (obj->inode->size <= MAX_SYMLINK_LEN_IN_INODE))) {
		obj->inode->size = size;
		obj->inode->mtime = obj->inode->ctime = time(NULL);
		obj->flags |= OFLAG_DIRTY;
		return EOK;
	}

	if (obj->inode->size > size) {
		/* Release contiguous runs of ALLOCATED blocks in one call each.
		 *
		 * A hole (*bno == 0) owns no storage, so it must break the run rather
		 * than extend it. The previous form tracked the last block number and
		 * tested `!lbno || (*bno == lbno + 1)`, which treated a hole as
		 * contiguous -- a hole set lbno = 0, and the next iteration's !lbno
		 * test then continued the run. The run was then released as
		 * `lbno + 1 - n`, i.e. `1 - n`, which UNDERFLOWS uint32_t for n > 1.
		 * That handed ext2_block_destroy() a huge block number, so
		 * ext2_blockToGroup() produced a huge group and the function read past
		 * the end of fs->gdt[]; and where it did not fault it freed blocks
		 * that had never been allocated, destroying whatever file owned them. */
		for (block = start; block < end; block++) {
			if ((err = ext2_block_get(fs, obj, block, &bno)) < 0)
				return err;

			if (*bno == 0) {
				if (run > 0) {
					if ((err = ext2_block_destroy(fs, first, run)) < 0)
						return err;

					freed += run;
					run = 0;
				}
				continue;
			}

			if (run == 0) {
				first = *bno;
				run = 1;
			}
			else if (*bno == (first + run)) {
				run++;
			}
			else {
				if ((err = ext2_block_destroy(fs, first, run)) < 0)
					return err;

				freed += run;
				first = *bno;
				run = 1;
			}
		}

		if (run > 0) {
			if ((err = ext2_block_destroy(fs, first, run)) < 0)
				return err;

			freed += run;
		}

		if ((err = ext2_iblock_destroy(fs, obj, start, end - start)) < 0)
			return err;

		/* Only what was actually released. The previous form subtracted the
		 * whole logical range, `(end - start) * blocksz / sectorsz`, counting
		 * holes that never held a block -- e2fsck: "i_blocks is 4294967282".
		 * It also sat outside this branch, so GROWING a file via truncate
		 * computed a negative (end - start) and underflowed too. Indirect
		 * blocks are accounted separately, by ext2_iblock_free(). */
		obj->inode->blocks -= freed * (fs->blocksz / fs->sectorsz);

		/* Zero what survives inside the last partial block.
		 *
		 * Whole blocks are released from `start` on, so when `size` falls
		 * inside a block the bytes between `size` and the end of that block
		 * stay on disk. Nothing reads them while the file is short, because
		 * _ext2_file_read() clamps to inode->size -- but extending the file
		 * again makes them readable, so previously-deleted content came back.
		 * POSIX requires that gap to read as zeros. */
		if ((size % fs->blocksz) != 0) {
			uint32_t last = size / fs->blocksz;
			uint32_t *lbno;

			if ((err = ext2_block_get(fs, obj, last, &lbno)) < 0)
				return err;

			/* A hole already reads as zeros and owns nothing to clear. */
			if (*lbno != 0) {
				void *data = malloc(fs->blocksz);
				if (data == NULL)
					return -ENOMEM;

				if ((err = ext2_block_init(fs, obj, last, data)) < 0) {
					free(data);
					return err;
				}

				memset((char *)data + (size % fs->blocksz), 0,
						fs->blocksz - (size % fs->blocksz));

				err = ext2_block_syncone(fs, obj, last, data);
				free(data);

				if (err < 0)
					return err;
			}
		}
	}

	obj->inode->size = size;
	obj->inode->mtime = obj->inode->ctime = time(NULL);
	obj->flags |= OFLAG_DIRTY;

	return EOK;
}

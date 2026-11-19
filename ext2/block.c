/*
 * Phoenix-RTOS
 *
 * EXT2 filesystem
 *
 * Block operations
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "block.h"
#include "inode.h"


/* ⚠ The byte offset MUST be computed in 64 bits.
 *
 * `bno` and `fs->blocksz` are both uint32_t, so `bno * fs->blocksz` is evaluated
 * in 32-bit arithmetic and WRAPS AT 4 GiB -- long before the result is widened
 * to the off_t the read/write callback takes. Every ext2 filesystem larger than
 * 4 GiB therefore read and wrote the wrong place, silently.
 *
 * Found 2026-09-20 on a 27.5 GiB ext2 USB partition: `ls` listed the root
 * directory correctly (low blocks) while a top-level directory in it would not
 * stat as a directory, because the Orlov allocator had put its inode in a block
 * group past the wrap. A 2 GiB partition on the same stick was perfect -- its
 * highest offset is 0x80000000, which still fits.
 *
 * Cast first, then multiply. */
/* Block number <-> block-bitmap position.
 *
 * A group's block bitmap is indexed from the filesystem's FIRST DATA BLOCK,
 * which ext2 defines as 1 when the block size is 1 KiB and 0 for every larger
 * block size. The bitmap helpers below take a 1-based bit index, so the two
 * conversions differ by `fstBlock - 1` -- which is zero only on a 1 KiB
 * filesystem. Folding that term in by hand is how every block a 4 KiB
 * filesystem allocated came to be off by one from the bit that was reserved
 * for it, so keep the conversion in exactly one place. */
static inline uint32_t ext2_blockToGroup(const ext2_t *fs, uint32_t bno)
{
	return (bno - fs->sb->fstBlock) / fs->sb->groupBlocks;
}


static inline uint32_t ext2_blockToOffs(const ext2_t *fs, uint32_t bno)
{
	return ((bno - fs->sb->fstBlock) % fs->sb->groupBlocks) + 1;
}


/* Is bno a block this filesystem actually has?
 *
 * An out-of-range block number means the caller derived it from corrupt
 * metadata or from arithmetic that underflowed. Nothing used to check, and
 * ext2_blockToGroup() of such a number yields a huge group index that reads
 * past the end of fs->gdt[] -- which is how a `1 - n` underflow in the
 * truncate path took the whole storage driver down with a Data Abort instead
 * of returning an error. Validating at the boundary keeps a bad number away
 * from the descriptor table whatever produced it. */
static inline int ext2_blockValid(const ext2_t *fs, uint32_t bno)
{
	return ((bno >= fs->sb->fstBlock) && (bno < fs->sb->blocks)) ? 1 : 0;
}


static inline uint32_t ext2_blockFromOffs(const ext2_t *fs, uint32_t group, uint32_t offs)
{
	return fs->sb->fstBlock + (group * fs->sb->groupBlocks) + offs - 1;
}



int ext2_block_read(ext2_t *fs, uint32_t bno, void *buff, uint32_t n)
{
	ssize_t size = n * fs->blocksz;
	off_t offs = (off_t)bno * (off_t)fs->blocksz;

	if (fs->strg != NULL) {
		if (fs->strg->dev->blk->ops->read(fs->strg, fs->strg->start + offs, buff, size) != size) {
			return -EIO;
		}
	}
	else if (fs->legacy.read != NULL) {
		if (fs->legacy.read(fs->legacy.devId, offs, buff, size) != size) {
			return -EIO;
		}
	}
	else {
		return -ENOSYS;
	}

	return EOK;
}


int ext2_block_write(ext2_t *fs, uint32_t bno, const void *buff, uint32_t n)
{
	ssize_t size = n * fs->blocksz;
	/* 64-bit offset, same reason as ext2_block_read above. */
	off_t offs = (off_t)bno * (off_t)fs->blocksz;

	if (fs->strg != NULL) {
		if (fs->strg->dev->blk->ops->write(fs->strg, fs->strg->start + offs, buff, size) != size) {
			return -EIO;
		}
	}
	else if (fs->legacy.write != NULL) {
		if (fs->legacy.write(fs->legacy.devId, offs, buff, size) != size) {
			return -EIO;
		}
	}
	else {
		return -ENOSYS;
	}

	return EOK;
}


/* Block allocation.
 *
 * Allocations are made in an in-memory copy of one group's block bitmap,
 * fs->balloc, and written to the device -- the bitmap, then the group
 * descriptor with its free count -- by ext2_block_flush(). One write() used to
 * write both for every allocation it made: one for each run of data blocks and
 * one for each indirect block, i.e. five bitmap and five descriptor writes per
 * MiB written in 1 MiB writes at 1 KiB blocks, where one of each will do.
 *
 * The rule that keeps this crash-safe is that a block is marked in the bitmap
 * on the device before any block pointer to it is: ext2_block_flush() runs
 * before every inode is written and before every indirect block is written,
 * except an indirect block that is itself among the allocations not yet
 * flushed -- nothing on the device points to that one yet, since writing the
 * pointer to it would have flushed first. So nothing on the device ever points
 * to a block its bitmap does not mark, whenever the power is cut.
 *
 * Allocations can stay pending after an operation returns, but only ones that
 * nothing on the device refers to: a write() writes its inode, which flushes
 * them, while reading a hole (which allocates its indirect blocks) or a write
 * that fails part way may not. Releasing blocks, allocating or releasing an
 * inode, sync and unmount flush them too. */


/* Writes the allocations made since the last flush to the device */
int ext2_block_flush(ext2_t *fs)
{
	uint32_t group;
	int err;

	if (fs->balloc.dirty == 0) {
		return EOK;
	}

	group = fs->balloc.group;

	err = ext2_block_write(fs, fs->gdt[group].blockBmp, fs->balloc.bmp, 1);
	if (err < 0) {
		return err;
	}

	memcpy(fs->balloc.disk, fs->balloc.bmp, fs->blocksz);

	/* On failure the allocations stay pending, and the next flush retries. */
	err = ext2_gdt_syncone(fs, group);
	if (err < 0) {
		return err;
	}

	fs->balloc.dirty = 0;

	return EOK;
}


/* Flushes the allocations and forgets the cached bitmap, for code that changes
 * a block bitmap on the device directly (releasing blocks). */
static int ext2_block_releaseBitmap(ext2_t *fs)
{
	int err = ext2_block_flush(fs);
	if (err < 0) {
		return err;
	}

	fs->balloc.valid = 0;

	return EOK;
}


/* Makes `group` the group allocations are made in, reading its bitmap. The
 * allocations pending in another group are flushed first. */
static int ext2_block_loadBitmap(ext2_t *fs, uint32_t group)
{
	int err;

	if ((fs->balloc.valid != 0) && (fs->balloc.group == group)) {
		return EOK;
	}

	err = ext2_block_flush(fs);
	if (err < 0) {
		return err;
	}

	if (fs->balloc.bmp == NULL) {
		char *buff = malloc(2u * fs->blocksz);
		if (buff == NULL) {
			return -ENOMEM;
		}

		fs->balloc.bmp = buff;
		fs->balloc.disk = buff + fs->blocksz;
	}

	fs->balloc.valid = 0;

	err = ext2_block_read(fs, fs->gdt[group].blockBmp, fs->balloc.disk, 1);
	if (err < 0) {
		return err;
	}

	memcpy(fs->balloc.bmp, fs->balloc.disk, fs->blocksz);
	fs->balloc.group = group;
	fs->balloc.valid = 1;

	return EOK;
}


/* Is bno allocated but not yet marked in the bitmap on the device? */
static int ext2_block_isPending(const ext2_t *fs, uint32_t bno)
{
	uint32_t offset;

	if ((fs->balloc.dirty == 0) || !ext2_blockValid(fs, bno) || (ext2_blockToGroup(fs, bno) != fs->balloc.group)) {
		return 0;
	}

	offset = ext2_blockToOffs(fs, bno);

	return ((ext2_checkbit(fs->balloc.bmp, offset) != 0) && (ext2_checkbit(fs->balloc.disk, offset) == 0)) ? 1 : 0;
}


/* Takes n free blocks of the loaded group, from offset on */
static void ext2_block_take(ext2_t *fs, uint32_t offset, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) {
		ext2_togglebit(fs->balloc.bmp, offset + i);
	}

	fs->balloc.dirty = 1;
	fs->gdt[fs->balloc.group].freeBlocks -= n;
	fs->sb->freeBlocks -= n;
}


/* Gives back n blocks taken by ext2_block_take() and not used */
static void ext2_block_untake(ext2_t *fs, uint32_t offset, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) {
		ext2_togglebit(fs->balloc.bmp, offset + i);
	}

	fs->gdt[fs->balloc.group].freeBlocks += n;
	fs->sb->freeBlocks += n;
}


int ext2_block_destroy(ext2_t *fs, uint32_t bno, uint32_t n)
{
	uint32_t group, i, j, pgroup, offset = 0;
	void *bmp;
	int err;

	if ((n == 0) || !ext2_blockValid(fs, bno) || ((fs->sb->blocks - bno) < n)) {
		return -EINVAL;
	}

	if ((err = ext2_block_releaseBitmap(fs)) < 0) {
		return err;
	}

	group = ext2_blockToGroup(fs, bno);
	pgroup = group;

	if ((bmp = malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	if ((err = ext2_block_read(fs, fs->gdt[group].blockBmp, bmp, 1)) < 0) {
		free(bmp);
		return err;
	}

	for (i = 0, j = 0; i < n; i++, j++) {
		group = ext2_blockToGroup(fs, bno + i);
		offset = ext2_blockToOffs(fs, bno + i);

		if (group != pgroup) {
			if ((err = ext2_block_write(fs, fs->gdt[pgroup].blockBmp, bmp, 1)) < 0) {
				free(bmp);
				return err;
			}

			fs->gdt[pgroup].freeBlocks += j;

			if ((err = ext2_gdt_syncone(fs, pgroup)) < 0) {
				for (i = 0; i < j; i++)
					ext2_togglebit(bmp, --offset);

				if (!ext2_block_write(fs, fs->gdt[pgroup].blockBmp, bmp, 1))
					fs->gdt[pgroup].freeBlocks -= j;
				else
					fs->sb->freeBlocks += j;

				free(bmp);
				return err;
			}

			fs->sb->freeBlocks += j;
			pgroup = group;
			j = 0;

			if ((err = ext2_block_read(fs, fs->gdt[group].blockBmp, bmp, 1)) < 0) {
				free(bmp);
				return err;
			}
		}

		ext2_togglebit(bmp, offset);
	}


	if ((err = ext2_block_write(fs, fs->gdt[group].blockBmp, bmp, 1)) < 0) {
		free(bmp);
		return err;
	}

	fs->gdt[group].freeBlocks += j;

	if ((err = ext2_gdt_syncone(fs, group)) < 0) {
		/* Roll the free back: re-mark every block used and write the bitmap
		 * again, so a failed descriptor sync does not leave blocks that the
		 * bitmap calls free but the group counts still call used. */
		for (i = 0; i < j; i++)
			ext2_togglebit(bmp, offset--);

		if (!ext2_block_write(fs, fs->gdt[group].blockBmp, bmp, 1))
			fs->gdt[group].freeBlocks -= j;
		else
			fs->sb->freeBlocks += j;

		free(bmp);
		return err;
	}

	fs->sb->freeBlocks += j;
	free(bmp);

	return EOK;
}


/* Can group `group` have a free block at all?
 *
 * The allocators below search group by group from a starting group, and used
 * to read every group's block bitmap to find out. The in-memory descriptor
 * already says when a group has none, so a full group is skipped without
 * touching the device.
 *
 * This is what made writing a large file slow. The search starts at the
 * file's home group for every allocation (about five per MiB of a file at
 * 1 KiB blocks: the data run plus one indirect block per 256 data blocks), so
 * every full group between the home group and the free space was read again
 * each time. Each bitmap lives in a different block group, i.e. a different
 * block-cache line, so each read was a cache miss and a whole cache-line read
 * from the card. On an SD root whose low groups are full (a new inode in group
 * 3, free space from group 181) that was over 900 line reads per MiB written.
 *
 * Skipping cannot make a group with free blocks unreachable: the count is
 * maintained together with the bitmap by every allocation and release below.
 * If the descriptor is wrong and says 0 for a group that has free bits, the
 * group is merely not used -- which is also what keeps that count from
 * wrapping below zero. */
static inline int ext2_groupHasFreeBlocks(const ext2_t *fs, uint32_t group)
{
	return (fs->gdt[group].freeBlocks != 0) ? 1 : 0;
}


/* Allocates one block, preferring the group that holds inode `ino` (its owner). */
static int ext2_block_createone(ext2_t *fs, uint32_t ino, uint32_t *res)
{
	uint32_t group = (ino - 1) / fs->sb->groupInodes;
	uint32_t offset, pgroup = group;
	int err;

	do {
		offset = 0;

		if (ext2_groupHasFreeBlocks(fs, group)) {
			if ((err = ext2_block_loadBitmap(fs, group)) < 0)
				return err;

			offset = ext2_findzerobit(fs->balloc.bmp, fs->sb->groupBlocks, 0);
		}

		if (!offset) {
			group = (group + 1) % fs->groups;

			if (group == pgroup)
				return -ENOSPC;
		}
	} while (!offset);

	ext2_block_take(fs, offset, 1);
	*res = ext2_blockFromOffs(fs, group, offset);

	return EOK;
}


/* Tries to allocate n consecutive blocks for logical blocks block..block + n - 1.
 * On success *res is the number allocated (at least one) and *first the
 * physical number of the first of them. */
static int ext2_block_create(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t lbno, uint32_t n, uint32_t *first, uint32_t *res)
{
	uint32_t *bno;
	int err;

	uint32_t group, offset;
	if (lbno != 0) {
		group = ext2_blockToGroup(fs, lbno);
		offset = ext2_blockToOffs(fs, lbno);
	}
	else {
		group = ((uint32_t)obj->id - 1) / fs->sb->groupInodes;
		offset = 0;
	}

	uint32_t startGroup = group;
	do {
		if (ext2_groupHasFreeBlocks(fs, group)) {
			err = ext2_block_loadBitmap(fs, group);
			if (err < 0) {
				return err;
			}

			if ((offset != 0) && (ext2_checkbit(fs->balloc.bmp, offset) == 0)) {
				break;
			}
			else {
				offset = ext2_findzerobit(fs->balloc.bmp, fs->sb->groupBlocks, offset);
			}
		}
		else {
			offset = 0;
		}

		if (offset == 0) {
			group = (group + 1) % fs->groups;
			if (group == startGroup) {
				return -ENOSPC;
			}
		}
	} while (offset == 0);

	uint32_t nFound;
	for (nFound = 0; (nFound < n) && ((offset + nFound) <= fs->sb->groupBlocks); nFound++) {
		if (ext2_checkbit(fs->balloc.bmp, offset + nFound) != 0) {
			break;
		}
	}

	if (nFound == 0) {
		return -ENOSPC;
	}

	/* Take the whole run before mapping it: mapping can allocate an indirect
	 * block, from this same bitmap, and it must not be given one of these. */
	ext2_block_take(fs, offset, nFound);

	for (uint32_t i = 0; i < nFound; i++) {
		err = ext2_block_get(fs, obj, block + i, &bno);
		if (err < 0) {
			/* Keep what is mapped already; give the rest back. If mapping
			 * allocated in another group, this one has been flushed, and the
			 * rest is released like any other block. */
			if (ext2_block_isPending(fs, ext2_blockFromOffs(fs, group, offset + i))) {
				ext2_block_untake(fs, offset + i, nFound - i);
			}
			else {
				(void)ext2_block_destroy(fs, ext2_blockFromOffs(fs, group, offset + i), nFound - i);
			}

			if (i == 0) {
				return err;
			}

			nFound = i;
			break;
		}

		*bno = ext2_blockFromOffs(fs, group, offset + i);
	}

	*first = ext2_blockFromOffs(fs, group, offset);
	*res = nFound;

	return EOK;
}


/* Calculates block indirection offsets */
static int ext2_block_offs(ext2_t *fs, uint32_t block, uint32_t offs[4])
{
	uint32_t addr = 256 << fs->sb->logBlocksz;
	uint32_t bits = 8 + fs->sb->logBlocksz;
	int depth = 0;

	if (block < DIRECT_BLOCKS) {
		offs[depth++] = block;
	}
	else if ((block -= DIRECT_BLOCKS) < addr) {
		offs[depth++] = block;
		offs[depth++] = SINGLE_INDIRECT_BLOCK;
	}
	else if ((block -= addr) < addr << bits) {
		offs[depth++] = block & (addr - 1);
		offs[depth++] = block >> bits;
		offs[depth++] = DOUBLE_INDIRECT_BLOCK;
	}
	else if ((block -= addr << bits) >> (2 * bits) < addr) {
		offs[depth++] = block & (addr - 1);
		offs[depth++] = (block >> bits) & (addr - 1);
		offs[depth++] = block >> (2 * bits);
		offs[depth++] = TRIPPLE_INDIRECT_BLOCK;
	}
	else {
		return -EINVAL;
	}

	return depth;
}


/* Reads an indirect block */
static int ext2_block_readind(ext2_t *fs, ext2_obj_t *obj, uint32_t *bno, int depth, uint32_t **ind, bool alloc)
{
	int err;

	depth -= 2;

	/* The truncate path clears these pointers as it frees each indirect block.
	 * Allocating a replacement for a pointer that has just been zeroed is what
	 * made every delete churn: free one block, allocate the next, forever. Only
	 * the WRITE path may create an indirect block. */
	if (!(*bno) && !alloc) {
		*ind = NULL;
		return EOK;
	}

	if (!(*bno) || (*bno != obj->ind[depth].bno)) {
		if (obj->ind[depth].data == NULL) {
			if ((obj->ind[depth].data = (uint32_t *)malloc(fs->blocksz)) == NULL)
				return -ENOMEM;
		}
		else {
			/* The block holds pointers, possibly to blocks allocated since the
			 * last flush; they must be marked on the device first, unless this
			 * block is itself one of them (then nothing points to it yet). */
			if (!ext2_block_isPending(fs, obj->ind[depth].bno) && ((err = ext2_block_flush(fs)) < 0))
				return err;

			if ((err = ext2_block_write(fs, obj->ind[depth].bno, obj->ind[depth].data, 1)) < 0)
				return err;
		}

		if (!(*bno)) {
			if ((err = ext2_block_createone(fs, (uint32_t)obj->id, &obj->ind[depth].bno)) < 0)
				return err;

			/* i_blocks counts a file's metadata blocks as well as its data;
			 * e2fsck expects the indirect blocks in the total. */
			obj->inode->blocks += fs->blocksz / fs->sectorsz;

			memset(obj->ind[depth].data, 0, fs->blocksz);
			*bno = obj->ind[depth].bno;
		}
		else {
			if ((err = ext2_block_read(fs, *bno, obj->ind[depth].data, 1)) < 0)
				return err;

			obj->ind[depth].bno = *bno;
		}
	}

	*ind = obj->ind[depth].data;

	return EOK;
}


/* Reads indirect blocks */
static int ext2_block_ind(ext2_t *fs, ext2_obj_t *obj, int depth, uint32_t offs[4], uint32_t *ind[3], bool alloc)
{
	int err;

	ind[0] = NULL;
	ind[1] = NULL;
	ind[2] = NULL;

	if (depth == 4) {
		if ((err = ext2_block_readind(fs, obj, obj->inode->block + offs[3], depth, ind + 2, alloc)) < 0)
			return err;
	}

	if (depth >= 3) {
		if (depth == 4) {
			/* With alloc == false a missing parent means the whole subtree is
			 * already gone; there is nothing below it to reach. */
			if (ind[2] == NULL) {
				return EOK;
			}

			if ((err = ext2_block_readind(fs, obj, ind[2] + offs[2], --depth, ind + 1, alloc)) < 0)
				return err;
		}
		else {
			if ((err = ext2_block_readind(fs, obj, obj->inode->block + offs[2], depth, ind + 1, alloc)) < 0)
				return err;
		}
	}

	if (depth >= 2) {
		if (depth == 3) {
			if (ind[1] == NULL) {
				return EOK;
			}

			if ((err = ext2_block_readind(fs, obj, ind[1] + offs[1], --depth, ind, alloc)) < 0)
				return err;
		}
		else {
			if ((err = ext2_block_readind(fs, obj, obj->inode->block + offs[1], depth, ind, alloc)) < 0)
				return err;
		}
	}

	return EOK;
}


int ext2_block_get(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t **res)
{
	uint32_t *ind[3] = { NULL, NULL, NULL };
	uint32_t offs[4] = { 0 };
	int err, depth;

	if ((depth = ext2_block_offs(fs, block, offs)) < 0)
		return depth;

	if (depth > 1) {
		if ((err = ext2_block_ind(fs, obj, depth, offs, ind, true)) < 0)
			return err;

		*res = ind[0] + offs[0];
	}
	else {
		*res = obj->inode->block + offs[0];
	}

	return EOK;
}


int ext2_block_syncone(ext2_t *fs, ext2_obj_t *obj, uint32_t block, const void *buff)
{
	uint32_t *bno;
	int err;

	if ((err = ext2_block_get(fs, obj, block, &bno)) < 0)
		return err;

	if (!(*bno)) {
		if ((err = ext2_block_createone(fs, (uint32_t)obj->id, &block)) < 0)
			return err;

		*bno = block;
		obj->inode->blocks += fs->blocksz / fs->sectorsz;
	}

	return ext2_block_write(fs, *bno, buff, 1);
}


int ext2_block_sync(ext2_t *fs, ext2_obj_t *obj, uint32_t block, const void *buff, uint32_t n)
{
	uint32_t lbno = 0, first = 0, i = 0, j = 0, k = 0;
	uint32_t *bno;
	int err;

	/* Blocks i..j - 1 are allocated and contiguous on disk, from physical block
	 * `first` to `lbno`, and not written yet.
	 *
	 * Physical numbers are kept by value. `bno` points into the cached indirect
	 * block, and the object caches one indirect block per level, so getting a
	 * block in another indirect block replaces it: the cached one is written
	 * back and the other read in. Getting block i again to find where its run
	 * starts did exactly that, and back again, whenever a run crossed from one
	 * indirect block to the next -- several times per MiB written. */
	while (j < n) {
		if ((err = ext2_block_get(fs, obj, block + j, &bno)) < 0)
			return err;

		if (*bno == 0) {
			if (i < j) {
				if ((err = ext2_block_write(fs, first, buff + i * fs->blocksz, j - i)) < 0)
					return err;
			}

			for (i = j++; j < n; j++) {
				if ((err = ext2_block_get(fs, obj, block + j, &bno)) < 0)
					return err;

				if (*bno)
					break;
			}

			for (; i < j; i += k) {
				if ((err = ext2_block_create(fs, obj, block + i, lbno, j - i, &first, &k)) < 0)
					return err;

				/* ext2_block_syncone() accounts for the single-block path; this
				 * is the bulk one and it did not, so every file written by an
				 * ordinary buffered write ended up with i_blocks == 0. */
				obj->inode->blocks += k * (fs->blocksz / fs->sectorsz);

				if ((err = ext2_block_write(fs, first, buff + i * fs->blocksz, k)) < 0)
					return err;
			}

			lbno = 0;
		}
		else if (lbno == 0) {
			/* The first block of a run */
			first = lbno = *bno;
			j++;
		}
		else if (*bno == lbno + 1) {
			lbno = *bno;
			j++;
		}
		else {
			/* This block is not contiguous with the run in progress, so it
			 * BEGINS the next one: write the run out and start a new one here.
			 *
			 * Setting lbno = 0 here instead left the new run's first block
			 * unrecorded: the next iteration saw !lbno, accepted whatever
			 * followed and set lbno from that, so block i and block i+1 were
			 * never compared. A run starting with two DISCONTIGUOUS blocks was
			 * then written as if contiguous, putting the second block's data
			 * on top of whatever lived after the first. */
			uint32_t cur = *bno;

			if ((err = ext2_block_write(fs, first, buff + i * fs->blocksz, j - i)) < 0)
				return err;

			first = lbno = cur;
			i = j++;
		}
	}

	if (i < j) {
		if ((err = ext2_block_write(fs, first, buff + i * fs->blocksz, j - i)) < 0)
			return err;
	}

	return EOK;
}


/* Destroys a block */
static int ext2_block_destroyone(ext2_t *fs, uint32_t bno)
{
	uint32_t group, offset;
	void *bmp;
	int err;

	if (!bno)
		return EOK;

	if (!ext2_blockValid(fs, bno)) {
		return -EINVAL;
	}

	if ((err = ext2_block_releaseBitmap(fs)) < 0) {
		return err;
	}

	group = ext2_blockToGroup(fs, bno);
	offset = ext2_blockToOffs(fs, bno);

	if ((bmp = malloc(fs->blocksz)) == NULL)
		return -ENOMEM;

	if ((err = ext2_block_read(fs, fs->gdt[group].blockBmp, bmp, 1)) < 0) {
		free(bmp);
		return err;
	}


	ext2_togglebit(bmp, offset);

	if ((err = ext2_block_write(fs, fs->gdt[group].blockBmp, bmp, 1)) < 0) {
		free(bmp);
		return err;
	}

	fs->gdt[group].freeBlocks++;

	if ((err = ext2_gdt_syncone(fs, group)) < 0) {
		ext2_togglebit(bmp, offset);

		if (!ext2_block_write(fs, fs->gdt[group].blockBmp, bmp, 1))
			fs->gdt[group].freeBlocks--;
		else
			fs->sb->freeBlocks++;

		free(bmp);
		return err;
	}

	fs->sb->freeBlocks++;
	free(bmp);

	return EOK;
}


/* Frees an indirect block AND drops it from i_blocks.
 *
 * ext2_block_readind() increments i_blocks when it allocates an indirect
 * block ("e2fsck expects the indirect blocks in the total"), but the release
 * path below only freed them, so every truncate left the file's i_blocks high
 * by exactly its indirect-block count -- e2fsck: "i_blocks is 18, should be
 * 0". Data blocks are accounted separately in _ext2_file_truncate(). */
static int ext2_iblock_free(ext2_t *fs, ext2_obj_t *obj, uint32_t bno)
{
	int err;

	if (bno == 0) {
		return EOK;
	}

	if ((err = ext2_block_destroyone(fs, bno)) < 0) {
		return err;
	}

	obj->inode->blocks -= fs->blocksz / fs->sectorsz;

	/* Drop the block from the indirect-block cache.
	 *
	 * ext2_block_readind() writes a cached indirect block back when it evicts
	 * it, and _ext2_obj_sync() writes back whatever is still cached. Freeing a
	 * block that one of those slots still names therefore let a stale copy be
	 * written to a block that had since been REALLOCATED to another file.
	 *
	 * The buffer is released rather than just clearing .bno: a slot with
	 * data != NULL and bno == 0 would make the eviction path write to block 0,
	 * and ext2_block_write() does not guard against that -- it would land on
	 * the superblock. With data == NULL the next use allocates afresh and
	 * takes the no-write-back path. */
	for (int d = 0; d < 3; d++) {
		if ((obj->ind[d].data != NULL) && (obj->ind[d].bno == bno)) {
			free(obj->ind[d].data);
			obj->ind[d].data = NULL;
			obj->ind[d].bno = 0;
		}
	}

	return EOK;
}


int ext2_iblock_destroy(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t n)
{
	uint32_t *ind[3] = { NULL, NULL, NULL };
	uint32_t i, offs[4] = { 0 };
	int err, depth;

	/* Walk the range BACKWARDS. Each case below releases an indirect block when
	 * its entry index reaches 0, meaning the block has just become empty, and
	 * zeroes the pointer to it. Going forwards, index 0 is the FIRST entry
	 * visited, so the pointer was cleared while the remaining 1023 entries were
	 * still to come -- and ext2_block_readind() allocates a new indirect block
	 * whenever it finds that pointer zero. Truncating therefore freed one
	 * indirect block and immediately allocated a replacement that nothing ever
	 * freed. Backwards, index 0 is the LAST entry visited, so nothing needs the
	 * pointer afterwards. */
	for (i = n; i-- > 0;) {
		if ((depth = ext2_block_offs(fs, block + i, offs)) < 0)
			return depth;

		if ((err = ext2_block_ind(fs, obj, depth, offs, ind, false)) < 0)
			return err;

		if (ind[0] != NULL)
			*(ind[0] + offs[0]) = 0;

		switch (depth) {
		case 1:
			obj->inode->block[offs[0]] = 0;
			break;

		case 2:
			if (!offs[0]) {
				if ((err = ext2_iblock_free(fs, obj, obj->inode->block[offs[1]])) < 0)
					return err;

				obj->inode->block[offs[1]] = 0;
			}
			break;

		/* A parent may only be released once its LAST child has been, so each
		 * level's test includes every level below it. Walking backwards,
		 * offs[k] == 0 means "this is the final entry of level k we will
		 * visit". Testing offs[1] alone freed the double indirect on the very
		 * first iteration -- offs[1] is already 0 when a file uses just one of
		 * its entries -- which zeroed the pointer and left the single indirect
		 * beneath it unreachable, and so never freed. */
		case 3:
			if (!offs[0] && (ind[1] != NULL)) {
				if ((err = ext2_iblock_free(fs, obj, *(ind[1] + offs[1]))) < 0)
					return err;

				*(ind[1] + offs[1]) = 0;
			}

			if (!offs[0] && !offs[1]) {
				if ((err = ext2_iblock_free(fs, obj, obj->inode->block[offs[2]])) < 0)
					return err;

				obj->inode->block[offs[2]] = 0;
			}
			break;

		case 4:
			if (!offs[0] && (ind[1] != NULL)) {
				if ((err = ext2_iblock_free(fs, obj, *(ind[1] + offs[1]))) < 0)
					return err;

				*(ind[1] + offs[1]) = 0;
			}

			if (!offs[0] && !offs[1] && (ind[2] != NULL)) {
				if ((err = ext2_iblock_free(fs, obj, *(ind[2] + offs[2]))) < 0)
					return err;

				*(ind[2] + offs[2]) = 0;
			}

			if (!offs[0] && !offs[1] && !offs[2]) {
				if ((err = ext2_iblock_free(fs, obj, obj->inode->block[offs[3]])) < 0)
					return err;

				obj->inode->block[offs[3]] = 0;
			}
			break;
		}
	}

	return EOK;
}


int ext2_block_init(ext2_t *fs, ext2_obj_t *obj, uint32_t block, void *buff)
{
	uint32_t *bno;
	int err;

	if ((err = ext2_block_get(fs, obj, block, &bno)) < 0)
		return err;

	/* An unallocated block is a HOLE, and a hole reads as zeros. Passing
	 * *bno == 0 to ext2_block_read() instead read block 0 OF THE DEVICE -- the
	 * boot sector and superblock -- so reading a sparse file's hole returned
	 * superblock bytes where POSIX requires zeros, and every freshly created
	 * file's first block began life as a copy of block 0 with only the written
	 * bytes laid over it. Those stale bytes become visible as soon as the file
	 * is extended. */
	if (*bno == 0) {
		memset(buff, 0, fs->blocksz);
		return EOK;
	}

	return ext2_block_read(fs, *bno, buff, 1);
}

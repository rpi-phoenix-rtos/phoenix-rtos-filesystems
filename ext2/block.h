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

#ifndef _BLOCK_H_
#define _BLOCK_H_

#include <stdint.h>

#include "ext2.h"


/* Reads blocks */
extern int ext2_block_read(ext2_t *fs, uint32_t bno, void *buff, uint32_t n);


/* Writes blocks */
extern int ext2_block_write(ext2_t *fs, uint32_t bno, const void *buff, uint32_t n);


/* Writes allocations made since the last call to the device: the block bitmap,
 * then its group descriptor. Must precede every write of a block pointer. */
extern int ext2_block_flush(ext2_t *fs);


/* Destroys blocks */
extern int ext2_block_destroy(ext2_t *fs, uint32_t bno, uint32_t n);


/* Finds the physical block number of an object's block (0 for a hole), without allocating anything */
extern int ext2_block_find(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t *res);


/* Returns where the physical block number of an object's block is stored,
 * allocating the indirect blocks on the way to it. Writers only: use
 * ext2_block_find() to look a block up. */
extern int ext2_block_get(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t **res);


/* Synchronizes one block (given object inode relative block number) */
extern int ext2_block_syncone(ext2_t *fs, ext2_obj_t *obj, uint32_t block, const void *buff);


/* Synchronizes blocks (given object inode relative block number). *written is
 * the number of leading blocks written, also when it fails part way. */
extern int ext2_block_sync(ext2_t *fs, ext2_obj_t *obj, uint32_t block, const void *buff, uint32_t n, uint32_t *written);


/* Destroys inode blocks (given object inode relative block number) */
extern int ext2_iblock_destroy(ext2_t *fs, ext2_obj_t *obj, uint32_t block, uint32_t n);


/* Initializes block (given object inode relative block number) */
extern int ext2_block_init(ext2_t *fs, ext2_obj_t *obj, uint32_t block, void *buff);


#endif

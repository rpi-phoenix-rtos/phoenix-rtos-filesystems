#
# Makefile for Phoenix-RTOS nfs filesystem server (#153 T2)
#
# Copyright 2026 Phoenix Systems
#
# %LICENSE%
#

LOCAL_PATH := $(call my-dir)

NAME := libnfsfs
SRCS := $(filter-out $(LOCAL_PATH)srv.c, $(wildcard $(LOCAL_PATH)*.c))
HEADERS := $(LOCAL_PATH)nfs_ops.h $(LOCAL_PATH)nfs_node.h
include $(static-lib.mk)

NAME := nfs
SRCS := $(LOCAL_PATH)srv.c
LIBS := libnfsfs
LOCAL_INSTALL_PATH := /sbin

# Link the libnfs port (T0). It declares no conflicts, so the port harness
# installs it into the flat sysroot (PREFIX_A/PREFIX_H), already on the global
# -L$(PREFIX_A)/-I$(PREFIX_H); a plain -lnfs resolves libnfs.a and
# <nfsc/libnfs.h>. The libnfs port (ports.yaml) must be built first.
LOCAL_LDLIBS := -lnfs

# The /dev/ifstatus parser copies fixed-width interface/IP fields with strncpy
# and NUL-terminates afterwards; -Wstringop-truncation flags the intentional
# no-trailing-NUL strncpy under -Werror. Downgrade just that one diagnostic
# (same as nfs-smoke's Makefile).
LOCAL_CFLAGS := -Wno-error=stringop-truncation
include $(binary.mk)

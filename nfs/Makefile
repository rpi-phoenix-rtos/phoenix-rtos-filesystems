#
# Makefile for Phoenix-RTOS nfs filesystem server (#153 T2)
#
# Copyright 2026 Phoenix Systems
#
# %LICENSE%
#

LOCAL_PATH := $(call my-dir)

# FS_DIAG must reach BOTH targets: nfs_ops.c compiles into this static library,
# and srv.c references symbols it defines under the flag. Setting it only on the
# binary target below leaves those symbols undefined and the link fails.
LOCAL_CFLAGS := $(FS_DIAG)

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
# FS_DIAG is a diagnostic -D hook for this server only (mirrors KERNEL_DIAG /
# LIBC_DIAG), e.g. FS_DIAG='-DNFS_MSG_TICK'. It must be APPENDED here rather
# than assigned earlier: this line is the last LOCAL_CFLAGS before
# include $(binary.mk) and a plain := earlier in the file is silently clobbered.
LOCAL_CFLAGS := -Wno-error=stringop-truncation $(FS_DIAG)
include $(binary.mk)

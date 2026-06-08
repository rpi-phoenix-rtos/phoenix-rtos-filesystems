/*
 * Phoenix-RTOS — NFS filesystem server (#153 T2 / T3b)
 *
 * A dummyfs-shaped userspace VFS server whose backing store is an NFS export
 * reached over the network via libnfs (the T0 port). It waits for a DHCP
 * lease, mounts the export, then splices itself under an existing directory
 * (e.g. /nfstest) via the mtSetAttr(atDev) mechanism — the same path the ia32
 * `dummyfs -m /tmp` mount and the ext2 SD-root use. It does NOT register "/"
 * (that is T3, the rootfs case).
 *
 * Single-threaded: the libnfs sync API drives one msgRecv loop. The loop and
 * the async splice both run on explicitly-sized >=64 KB stacks (the #120
 * pool-thread-stack-overflow lesson: the NFS call chain msgRecv -> handler ->
 * libnfs sync -> XDR -> socket-to-lwip is deeper than ext2-over-SD).
 *
 * Usage (argv): nfs <mountpoint> [server-ip] [export] [v3|v4] [root|takeover]
 *   defaults:   nfs /nfstest 10.42.0.1 / v4
 *
 * Root mode (a trailing "root" token, #153 T3): the NFS export becomes "/"
 * itself, registered BEFORE any RAM "/" exists. The server accepts "/" as the
 * mountpoint, skips the fopen("/dev/ifstatus") DHCP-wait (unusable pre-"/"),
 * bounded-retries nfs_init_context+nfs_mount until DHCP lands (or a deadline),
 * and portRegister("/")s the export directly. Example: nfs / 10.42.0.1 / v4
 * root. (Blocked in practice by the kernel pre-"/" name-resolver gap, #153 T3
 * Gap B — kept for reference; design-A/takeover is the working path.)
 *
 * Takeover mode (a trailing "takeover" token, #153 T3 design-A): the NFS
 * export becomes "/", but AFTER a normal dummyfs RAM "/" + /dev bind + lwip
 * have come up. Because "/" already exists, sockets resolve normally and we
 * reuse the SAME proven subtree path as the /nfstest mount: the normal
 * /dev/ifstatus DHCP-wait, then nfs_makeContext + nfs_mount. We then TAKE OVER
 * "/": first try the mtSetAttr(atDev) splice onto the "/" oid (the same
 * mechanism the /nfstest mount uses), re-resolve "/" to see whether the kernel
 * honored it, and if not fall back to portUnregister("/") + portRegister("/").
 * Example: nfs / 10.42.0.1 / v4 takeover.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/threads.h>

#include <nfsc/libnfs.h>

#include "nfs_ops.h"

/* NFS_V3/NFS_V4 live in libnfs's internal RPC headers, not the public <nfsc/>
 * set the port installs; define the stable wire program versions locally (as
 * nfs-smoke does). */
#ifndef NFS_V3
#define NFS_V3 3
#endif
#ifndef NFS_V4
#define NFS_V4 4
#endif

#define LOG(fmt, ...) printf("nfs-fs: " fmt, ##__VA_ARGS__)

/* 16 * _PAGE_SIZE = 64 KB; the deep NFS handler call chain overflows the 8 KB
 * default pool-thread stack (#120). Applies to BOTH the loop and the splice. */
#define NFS_STACKSZ (16 * _PAGE_SIZE)


static struct {
	nfs_fs_t fs;
	const char *mountpt;
	char __attribute__((aligned(8))) loopStack[NFS_STACKSZ];
	char __attribute__((aligned(8))) mountStack[NFS_STACKSZ];
} common;


static int valid_ipv4(const char *s)
{
	int parts = 0, digits = 0, val = 0, nonzero = 0;
	for (;; s++) {
		if (*s >= '0' && *s <= '9') {
			val = val * 10 + (*s - '0');
			digits++;
			if (val > 255) {
				return 0;
			}
		}
		else if (*s == '.' || *s == '\0') {
			if (digits == 0) {
				return 0;
			}
			if (val != 0) {
				nonzero = 1;
			}
			parts++;
			if (*s == '\0') {
				break;
			}
			val = 0;
			digits = 0;
		}
		else {
			return 0;
		}
	}
	return (parts == 4 && nonzero) ? 1 : 0;
}


/* Scan /dev/ifstatus for ANY non-lo/wl/sc interface with a bound IPv4.
 * Interface-name-agnostic (Pi4 en0, ia32-qemu en1, ...) — copied from
 * nfs-smoke so the fs server has no nfs-smoke dependency. */
static int wait_for_dhcp_lease(char *ip_out, size_t cap, int timeout_ms)
{
	int waited = 0;
	for (;;) {
		FILE *f = fopen("/dev/ifstatus", "r");
		if (f != NULL) {
			char line[160];
			char cur_if[24] = "";
			int cur_up = 0;
			while (fgets(line, sizeof(line), f) != NULL) {
				char *us = strchr(line, '_');
				char *eq = strchr(line, '=');
				if (us == NULL || eq == NULL || us > eq) {
					continue;
				}
				size_t iflen = (size_t)(us - line);
				char ifname[24];
				if (iflen == 0 || iflen >= sizeof(ifname)) {
					continue;
				}
				memcpy(ifname, line, iflen);
				ifname[iflen] = '\0';

				char key[24];
				size_t keylen = (size_t)(eq - (us + 1));
				if (keylen == 0 || keylen >= sizeof(key)) {
					continue;
				}
				memcpy(key, us + 1, keylen);
				key[keylen] = '\0';

				char val[64];
				strncpy(val, eq + 1, sizeof(val) - 1);
				val[sizeof(val) - 1] = '\0';
				char *nl = strpbrk(val, "\r\n");
				if (nl != NULL) {
					*nl = '\0';
				}

				int is_lo = (strncmp(ifname, "lo", 2) == 0) || (strncmp(ifname, "wl", 2) == 0) || (strncmp(ifname, "sc", 2) == 0);
				if (is_lo) {
					continue;
				}

				if (strcmp(key, "up") == 0) {
					strncpy(cur_if, ifname, sizeof(cur_if) - 1);
					cur_if[sizeof(cur_if) - 1] = '\0';
					cur_up = (atoi(val) != 0);
				}
				else if (strcmp(key, "ip") == 0) {
					if (cur_up && strcmp(ifname, cur_if) == 0 && valid_ipv4(val)) {
						strncpy(ip_out, val, cap - 1);
						ip_out[cap - 1] = '\0';
						fclose(f);
						return 0;
					}
				}
			}
			fclose(f);
		}
		if (waited >= timeout_ms) {
			return -1;
		}
		usleep(250000);
		waited += 250;
	}
}


/* Async splice (mirrors dummyfs_do_mount). Runs on its own >=64 KB stack and
 * spins until "/" is registered before sending the mtSetAttr(atDev) to the
 * parent fs. Also records the parent oid so ".." at the export root resolves. */
static void nfs_mountThread(void *arg)
{
	const char *mountpt = (const char *)arg;
	oid_t parent;
	oid_t self = { .port = common.fs.port, .id = NFS_ROOTID };
	struct stat buf;
	msg_t msg = { 0 };

	/* Wait for the root fs to come up. */
	while ((lookup("/", NULL, &parent) < 0) || (parent.port == common.fs.port)) {
		usleep(100000);
	}

	if (lookup(mountpt, &parent, NULL) < 0) {
		LOG("mountpoint %s not found (mkdir it first)\n", mountpt);
		endthread();
	}

	if (stat(mountpt, &buf) != 0 || !S_ISDIR(buf.st_mode)) {
		LOG("mountpoint %s is not a directory\n", mountpt);
		endthread();
	}

	/* Record the parent fs that owns the mountpoint dir so lookup("..") at
	 * our root crosses back into it. */
	common.fs.parent = parent;

	msg.type = mtSetAttr;
	msg.oid = parent;
	msg.i.attr.type = atDev;
	msg.i.data = &self;
	msg.i.size = sizeof(oid_t);

	int err = msgSend(parent.port, &msg);
	if (err < 0 || msg.o.err < 0) {
		LOG("splice at %s failed (err=%d)\n", mountpt, (err < 0) ? err : msg.o.err);
	}
	else {
		LOG("mounted at %s\n", mountpt);
	}

	endthread();
}


static void nfs_loopThread(void *arg)
{
	(void)arg;
	nfs_fs_t *fs = &common.fs;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(fs->port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = nfs_ops_open(fs, &msg.oid);
				break;

			case mtClose:
				msg.o.err = nfs_ops_close(fs, &msg.oid);
				break;

			case mtRead:
				msg.o.err = nfs_ops_read(fs, &msg.oid, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = nfs_ops_write(fs, &msg.oid, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtTruncate:
				msg.o.err = nfs_ops_truncate(fs, &msg.oid, msg.i.io.len);
				break;

			case mtDevCtl:
				msg.o.err = -EINVAL;
				break;

			case mtCreate:
				msg.o.err = nfs_ops_create(fs, &msg.oid, msg.i.data, &msg.o.create.oid, msg.i.create.mode, msg.i.create.type, &msg.i.create.dev);
				break;

			case mtDestroy:
				msg.o.err = nfs_ops_destroy(fs, &msg.oid);
				break;

			case mtSetAttr:
				msg.o.err = nfs_ops_setattr(fs, &msg.oid, msg.i.attr.type, msg.i.attr.val, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.err = nfs_ops_getattr(fs, &msg.oid, msg.i.attr.type, &msg.o.attr.val);
				break;

			case mtGetAttrAll: {
				struct _attrAll *attrs = msg.o.data;
				if ((attrs == NULL) || (msg.o.size < sizeof(struct _attrAll))) {
					msg.o.err = -EINVAL;
				}
				else {
					msg.o.err = nfs_ops_getattrAll(fs, &msg.oid, attrs);
				}
				break;
			}

			case mtLookup:
				msg.o.err = nfs_ops_lookup(fs, &msg.oid, msg.i.data, &msg.o.lookup.fil, &msg.o.lookup.dev);
				break;

			case mtLink:
				msg.o.err = nfs_ops_link(fs, &msg.oid, msg.i.data, &msg.i.ln.oid);
				break;

			case mtUnlink:
				msg.o.err = nfs_ops_unlink(fs, &msg.oid, msg.i.data);
				break;

			case mtReaddir:
				msg.o.err = nfs_ops_readdir(fs, &msg.oid, msg.i.readdir.offs, msg.o.data, msg.o.size);
				break;

			case mtStat:
				msg.o.err = nfs_ops_statfs(fs, msg.o.data, msg.o.size);
				break;

			default:
				msg.o.err = -EINVAL;
				break;
		}

		msgRespond(fs->port, &msg, rid);
	}
}


/* (Re-)create a libnfs context with our fixed transfer parameters. Used by the
 * root-mode bounded-retry loop, which throws away and rebuilds the context on
 * each failed mount attempt. */
static struct nfs_context *nfs_makeContext(int version)
{
	struct nfs_context *nfs = nfs_init_context();
	if (nfs == NULL) {
		return NULL;
	}
	nfs_set_version(nfs, version);
	nfs_set_timeout(nfs, 5000); /* bound every RPC so one drop can't wedge the loop */
	nfs_set_readmax(nfs, 32 * 1024);
	nfs_set_writemax(nfs, 32 * 1024);
	return nfs;
}


/* Root mode (#153 T3): mount the NFS export and portRegister it AS "/".
 *
 * NOT FUNCTIONAL on the current kernel — kept for reference only; the working
 * NFS-as-root path is takeover mode (nfs_runTakeover) below. Root mode founders
 * on socket() name resolution pre-"/": the kernel resolver does not forward a
 * "devfs/<name>" mtLookup to the registered "devfs" named port before a root
 * "/" exists (T3 finding, docs/research/.../T3-attempt-1-result.md UPDATE 3),
 * so libnfs cannot open its socket during nfs_mount and the export never gets
 * registered as "/". Fixing that would be a core kernel namespace change;
 * takeover mode sidesteps it by mounting after a RAM "/" already exists.
 *
 * The mechanics below are retained as the reference design: pre-"/" we cannot
 * fopen("/dev/ifstatus") to wait for DHCP, so we settle 10 s then bounded-retry
 * nfs_init_context+nfs_mount with a 3 s backoff until success or a ~90 s
 * deadline. On a successful mount we portRegister(port, "/") directly (mirroring
 * the dummyfs root path and the SD ext2-root). We do NOT start nfs_mountThread:
 * its mtSetAttr(atDev) splice waits for an *existing* "/", which would deadlock
 * when we ARE "/". parent stays {own-port, NFS_ROOTID} (self), so ".." at "/"
 * stays at "/". */
static int nfs_runRoot(const char *server, const char *export, const char *verstr, int version)
{
	const int deadline_s = 90;
	time_t start = time(NULL);
	int attempt = 0;

	LOG("root mode: mounting %s:%s as / (bounded retry, %ds deadline)\n", server, export, deadline_s);

	/* Initial settle: in root mode we skip the /dev/ifstatus DHCP-wait (the node
	 * does not exist pre-"/"), so give lwip time to bring up genet + complete its
	 * own DHCP before any socket() traffic. Hammering nfs_init_context+nfs_mount
	 * at ~1 Hz immediately starves lwip's tcpip thread during its DHCP window. */
	LOG("settling 10s before first mount (lets lwip finish DHCP)\n");
	sleep(10);

	for (;;) {
		struct nfs_context *nfs = nfs_makeContext(version);
		if (nfs != NULL) {
			if (nfs_mount(nfs, server, export) == 0) {
				common.fs.nfs = nfs;
				break;
			}
			LOG("root mount attempt %d failed: %s\n", attempt, nfs_get_error(nfs));
			nfs_destroy_context(nfs);
		}
		else {
			LOG("root mount attempt %d: nfs_init_context returned NULL\n", attempt);
		}

		if ((time(NULL) - start) >= deadline_s) {
			LOG("FATAL root mount failed after %ds, / not registered\n", deadline_s);
			return 2;
		}
		attempt++;
		usleep(3000000); /* 3 s inter-attempt backoff: reduce the socket() storm into the not-yet-ready socketsrv */
	}

	LOG("root mode: mounted %s:%s via %s after %d retr%s\n", server, export, verstr, attempt, (attempt == 1) ? "y" : "ies");

	if (nfs_node_init(&common.fs.nodes) != 0) {
		LOG("FAIL node table init\n");
		return 5;
	}

	if (portCreate(&common.fs.port) != 0) {
		LOG("FAIL portCreate\n");
		return 6;
	}

	/* "/" has no parent — self-parent so ".." at "/" stays at "/" (POSIX). */
	common.fs.parent.port = common.fs.port;
	common.fs.parent.id = NFS_ROOTID;

	oid_t root = { .port = common.fs.port, .id = NFS_ROOTID };
	if (portRegister(common.fs.port, "/", &root) < 0) {
		LOG("FATAL portRegister(/) failed, / not registered\n");
		return 7;
	}
	LOG("registered / (root mode)\n");

	/* Run the message loop on its own >=64 KB stack (the primary stack may be
	 * the 8 KB default). No splice thread in root mode. */
	beginthread(nfs_loopThread, 4, common.loopStack, sizeof(common.loopStack), NULL);

	for (;;) {
		usleep(1000000);
	}

	return 0;
}


/* Takeover mode (#153 T3 design-A): mount the NFS export and make it "/" AFTER
 * a normal dummyfs RAM "/" already exists. Reuses the proven subtree path
 * (normal /dev/ifstatus DHCP-wait + nfs_makeContext + nfs_mount — sockets
 * resolve normally because "/" and /dev are up), then takes over "/".
 *
 * Takeover mechanism + the runtime decision between the two paths:
 *   1. Resolve the current "/" oid (the dummyfs root) and try the mtSetAttr
 *      (atDev) splice onto it — the same splice the /nfstest mount uses, just
 *      targeting "/". Then re-resolve "/": if it now points at OUR port the
 *      splice took, log "via splice". (In practice the kernel returns the
 *      registered rootOid for a bare "/" lookup without consulting the root
 *      node's atDev, name.c:239-256, so the splice no-ops for "/" and the
 *      fallback below is what actually fires — we still try it first + decide
 *      at runtime so the log reflects reality on any kernel.)
 *   2. Fallback: portUnregister("/") then portRegister(port,"/"). This is the
 *      definitive takeover (proc_portUnregister clears rootRegistered, then
 *      proc_portRegister installs our oid as rootOid). */
static int nfs_runTakeover(const char *server, const char *export, const char *verstr, int version)
{
	char ipbuf[64] = "";

	/* "/" already exists, so the normal /dev/ifstatus DHCP-wait works (unlike
	 * the pre-"/" root mode). Reuse the subtree-mode wait. */
	/* Failures up to the portUnregister("/") below are non-destructive: the
	 * dummyfs RAM "/" is still registered, so when this process exits the system
	 * keeps booting on the (sparse) RAM root rather than bricking with no root.
	 * These are graceful degrades, not fatal aborts — say so in the log. */
	if (wait_for_dhcp_lease(ipbuf, sizeof(ipbuf), 30000) != 0) {
		LOG("takeover aborted: no DHCP lease in 30s; keeping RAM root /\n");
		return 2;
	}
	LOG("takeover: interface bound, ip=%s\n", ipbuf);

	common.fs.nfs = nfs_makeContext(version);
	if (common.fs.nfs == NULL) {
		LOG("takeover aborted: nfs_init_context failed; keeping RAM root /\n");
		return 3;
	}

	if (nfs_mount(common.fs.nfs, server, export) != 0) {
		LOG("takeover aborted: mount %s:%s: %s; keeping RAM root /\n", server, export, nfs_get_error(common.fs.nfs));
		nfs_destroy_context(common.fs.nfs);
		return 4;
	}
	/* Same marker the subtree mount prints — the orchestrator reads this first:
	 * it proves sockets work with a dummyfs "/" up (the design-A premise). */
	LOG("mounted %s:%s via %s\n", server, export, verstr);

	if (nfs_node_init(&common.fs.nodes) != 0) {
		LOG("takeover aborted: node table init failed; keeping RAM root /\n");
		return 5;
	}

	if (portCreate(&common.fs.port) != 0) {
		LOG("takeover aborted: portCreate failed; keeping RAM root /\n");
		return 6;
	}

	/* "/" has no parent — self-parent so ".." at "/" stays at "/" (POSIX). */
	common.fs.parent.port = common.fs.port;
	common.fs.parent.id = NFS_ROOTID;

	oid_t self = { .port = common.fs.port, .id = NFS_ROOTID };

	/* Re-bind /dev onto the NFS root, IN-PROCESS, before we become "/".
	 *
	 * The boot script's `bind devfs /dev` registered /dev in the *dummyfs*
	 * root's namespace; once we own "/", that bind is in the old root and /dev
	 * would be empty here. The device nodes themselves live in the "devfs"
	 * named-port process (a kernel-dcache name, untouched by the root swap), so
	 * all we must do is splice that devfs port onto OUR /dev node — exactly
	 * what `bind devfs /dev` does (mtSetAttr(atDev)), but done in-process so
	 * there is no second `bind` program and no race: by the time we register
	 * "/", /dev already resolves. (Equivalent to the kernel mount machinery;
	 * uses the node->mnt field added for #153 T3 design-A.) */
	oid_t devfsOid;
	if (lookup("devfs", NULL, &devfsOid) == 0) {
		/* Materialize /dev on the export if absent (EEXIST is fine). */
		(void)nfs_mkdir2(common.fs.nfs, "/dev", 0755);
		nfs_node_t *devNode = nfs_node_get(&common.fs.nodes, "/dev");
		if (devNode != NULL) {
			devNode->type = otDir;
			devNode->mnt = devfsOid;
			LOG("re-bound /dev (takeover, devfs port=%u)\n", devfsOid.port);
		}
		else {
			LOG("re-bind /dev: node alloc failed (devfs at /dev will be empty)\n");
		}
	}
	else {
		LOG("re-bind /dev: devfs port not found (devfs at /dev will be empty)\n");
	}

	/* Start serving BEFORE the takeover so the new "/" answers lookups the
	 * instant it is installed (no window where "/" resolves to a dead port). */
	beginthread(nfs_loopThread, 4, common.loopStack, sizeof(common.loopStack), NULL);

	/* Path 1: try the proven mtSetAttr(atDev) splice onto the existing "/". */
	oid_t oldRoot;
	int tookOver = 0;
	if (lookup("/", NULL, &oldRoot) == 0) {
		msg_t msg = { 0 };
		msg.type = mtSetAttr;
		msg.oid = oldRoot;
		msg.i.attr.type = atDev;
		msg.i.data = &self;
		msg.i.size = sizeof(oid_t);
		int err = msgSend(oldRoot.port, &msg);
		int spliceRc = (err < 0) ? err : msg.o.err;

		/* Decide at runtime: did the splice make "/" resolve to OUR port? */
		oid_t check;
		if ((spliceRc == 0) && (lookup("/", NULL, &check) == 0) && (check.port == common.fs.port)) {
			LOG("takeover via splice rc=%d\n", spliceRc);
			tookOver = 1;
		}
		else {
			LOG("takeover via splice rc=%d (not honored for /, falling back to portRegister)\n", spliceRc);
		}
	}
	else {
		LOG("takeover: could not resolve existing / for splice, falling back to portRegister\n");
	}

	/* Path 2 (fallback): unregister the old "/" then register ours. This is the
	 * one destructive window — between portUnregister and a successful
	 * portRegister there is briefly no "/". Capture the previous root oid first
	 * so that if our portRegister fails we can restore it and degrade to the RAM
	 * root instead of leaving the system with no "/" at all. proc_portRegister
	 * accepts any port for "/" (no ownership check) and only rejects when one is
	 * already registered, so this restore is also safe if portUnregister itself
	 * failed (rootRegistered stays set -> our register -EEXISTs -> restore
	 * -EEXISTs, "/" still resolves to the dummyfs root). Defensive: portRegister
	 * post-unregister does not fail in practice, so this branch is unexercised. */
	if (tookOver == 0) {
		oid_t prevRoot;
		int havePrev = (lookup("/", NULL, &prevRoot) == 0);
		int urc = portUnregister("/");
		int prc = portRegister(common.fs.port, "/", &self);
		LOG("takeover via portRegister rc=%d (unregister rc=%d)\n", prc, urc);
		if (prc < 0) {
			if (havePrev != 0) {
				int rrc = portRegister(prevRoot.port, "/", &prevRoot);
				LOG("takeover failed: portRegister(/) rc=%d; restored RAM root (rc=%d)\n", prc, rrc);
			}
			else {
				LOG("takeover failed: portRegister(/) rc=%d; no prior root captured to restore\n", prc);
			}
			return 7;
		}
	}

	LOG("registered / (takeover)\n");

	/* The loop thread serves "/" forever; park the main thread. */
	for (;;) {
		usleep(1000000);
	}

	return 0;
}


int main(int argc, char **argv)
{
	const char *mountpt = (argc > 1) ? argv[1] : "/nfstest";
	const char *server = (argc > 2) ? argv[2] : "10.42.0.1";
	const char *export = (argc > 3) ? argv[3] : "/";
	const char *verstr = (argc > 4) ? argv[4] : "v4";
	/* A trailing token selects a special mode: "root" registers the export as
	 * "/" pre-"/"; "takeover" mounts it as "/" after a RAM "/" already exists. */
	int rootMode = (argc > 5) && (strcmp(argv[5], "root") == 0);
	int takeoverMode = (argc > 5) && (strcmp(argv[5], "takeover") == 0);
	char ipbuf[64] = "";

	int version = (strcmp(verstr, "v3") == 0) ? NFS_V3 : NFS_V4;

	LOG("start (mountpt=%s server=%s export=%s %s%s%s)\n", mountpt, server, export, verstr,
		rootMode ? " root" : "", takeoverMode ? " takeover" : "");

	if (rootMode) {
		return nfs_runRoot(server, export, verstr, version);
	}

	if (takeoverMode) {
		return nfs_runTakeover(server, export, verstr, version);
	}

	if (mountpt[0] != '/' || strcmp(mountpt, "/") == 0) {
		LOG("refusing to register '/' (that is the root case — pass the trailing 'root' token); give a subtree like /nfstest\n");
		return 1;
	}

	if (wait_for_dhcp_lease(ipbuf, sizeof(ipbuf), 30000) != 0) {
		LOG("FAIL no DHCP lease in 30s\n");
		return 2;
	}
	LOG("interface bound, ip=%s\n", ipbuf);

	common.fs.nfs = nfs_makeContext(version);
	if (common.fs.nfs == NULL) {
		LOG("FAIL nfs_init_context\n");
		return 3;
	}

	if (nfs_mount(common.fs.nfs, server, export) != 0) {
		LOG("FAIL mount %s:%s: %s\n", server, export, nfs_get_error(common.fs.nfs));
		nfs_destroy_context(common.fs.nfs);
		return 4;
	}
	LOG("mounted %s:%s via %s\n", server, export, verstr);

	if (nfs_node_init(&common.fs.nodes) != 0) {
		LOG("FAIL node table init\n");
		return 5;
	}

	if (portCreate(&common.fs.port) != 0) {
		LOG("FAIL portCreate\n");
		return 6;
	}

	/* Default parent until the splice records the real one. */
	common.fs.parent.port = common.fs.port;
	common.fs.parent.id = NFS_ROOTID;
	common.mountpt = mountpt;

	/* Splice ourselves under the existing mountpoint dir (async: "/" may not
	 * be registered yet at launch). */
	beginthread(nfs_mountThread, 4, common.mountStack, sizeof(common.mountStack), (void *)mountpt);

	/* Run the message loop on its own >=64 KB stack (the primary stack may be
	 * the 8 KB default). */
	LOG("initialized\n");
	beginthread(nfs_loopThread, 4, common.loopStack, sizeof(common.loopStack), NULL);

	/* The two worker threads run forever; park the main thread. */
	for (;;) {
		usleep(1000000);
	}

	return 0;
}

// SPDX-License-Identifier: GPL-2.0
/*
 * Query running isp_app PID and its argv (same as ps aux | grep isp_app + read args).
 * Map argument patterns to allowed armcb_isp_v4l2-00..03 (ctx_id bit mask).
 *
 * Patterns (matched anywhere in argv string after normalization):
 *  1) -c | -s 0 | -m 0     -> only -00
 *  2) *-s 1*               -> only -01  (e.g. -s 1, -s 10)
 *  3) *-s 2*               -> only -02
 *  4) *-s 3*               -> only -03
 *  5) *-m 2*               -> -00 and -01
 *  6) *-m 3*               -> -00, -01, -02
 *  7) *-m 4*               -> -00..-03; multi-open: open -00 first
 *  8) *-m 1*               -> only -00
 *
 * Parse priority: -m before -s before -c.  Multiple isp_app: largest PID wins.
 *
 * No isp_app process in the system: only the current task whose comm is isp_app
 * may open video nodes (all ctx 0..3); any other opener is denied.
 *
 * isp_app always may open ctx 0..3 (enumerate /dev/video* and QUERYCAP during
 * startup). Cmdline mask applies only to non-isp_app openers once policy is
 * loaded. Full process scan is not run during isp_app open (avoids reentrancy).
 */

#include <linux/atomic.h>
#include <linux/jhash.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "system_logger.h"
#include "armcb_isp_execstart.h"

#ifdef LOG_MODULE
#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_ISP
#endif

#define ISP_PROCESS_NAME "isp_app"
#define ISP_CMDLINE_READ_MAX 4096

struct isp_app_info {
	int pid;
	char cmdline[ISP_CMDLINE_READ_MAX];
};

static DEFINE_MUTEX(armcb_isp_execstart_lock);
static unsigned int armcb_isp_allowed_ctx_mask;
static int armcb_isp_app_pid;
static u64 armcb_isp_process_cache_stamp;
static bool armcb_isp_process_cache_valid;
static bool armcb_isp_daemon_running;
static atomic_t armcb_isp_ctx0_open_refs = ATOMIC_INIT(0);

static bool execstart_ignore;

module_param(execstart_ignore, bool, 0644);
MODULE_PARM_DESC(execstart_ignore,
		 "If true, skip isp_app cmdline policy (allow ctx 0-3, no -00-first rule).");

/* Same comm match as armcb_v4l2_core find_user_process_by_name("isp_app"). */
static bool current_opener_is_isp_app(void)
{
	struct task_struct *curr = current;

	if (!curr->mm)
		return false;
	return !strncmp(curr->comm, ISP_PROCESS_NAME, 7);
}

static int execstart_isp_app_pid_get(char *buf, const struct kernel_param *kp)
{
	int pid;

	mutex_lock(&armcb_isp_execstart_lock);
	pid = armcb_isp_app_pid;
	mutex_unlock(&armcb_isp_execstart_lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", pid);
}

static const struct kernel_param_ops execstart_isp_app_pid_ops = {
	.get = execstart_isp_app_pid_get,
};

module_param_cb(execstart_isp_app_pid, &execstart_isp_app_pid_ops, NULL, 0444);
MODULE_PARM_DESC(execstart_isp_app_pid,
		 "Read-only: PID of isp_app used for cmdline policy (0 if none).");

static unsigned int leading_digit(unsigned int val)
{
	while (val >= 10)
		val /= 10;
	return val;
}

/* Option must not be part of a longer flag (e.g. not "--mode"). */
static bool is_short_option(const char *cmdline, const char *opt, size_t opt_len)
{
	if (opt > cmdline && opt[-1] != ' ' && opt[-1] != '\t')
		return false;
	if (opt[opt_len] != '\0' && opt[opt_len] != ' ' && opt[opt_len] != '\t' &&
	    (opt[opt_len] < '0' || opt[opt_len] > '9'))
		return false;
	return true;
}

/*
 * After "-m" or "-s", parse unsigned value; supports "-s 1" and "-s1".
 * Returns leading digit of value, or -1 on failure.
 */
static int parse_option_leading_digit(const char *cmdline, const char *opt)
{
	unsigned int val;
	int n;

	opt += 2;
	while (*opt == ' ' || *opt == '\t')
		opt++;
	if (*opt < '0' || *opt > '9')
		return -1;
	if (sscanf(opt, "%u%n", &val, &n) < 1)
		return -1;
	return (int)leading_digit(val);
}

/*
 * Match argv patterns 1..8.  * in the spec means substring anywhere in cmdline.
 */
static int isp_app_args_to_mask(const char *cmdline)
{
	const char *p;
	int ld;

	if (!cmdline || !*cmdline)
		return -EINVAL;

	/* Rules 5/6/7/8 and rule 1 (-m 0): *-m N* */
	p = cmdline;
	while ((p = strstr(p, "-m"))) {
		if (!is_short_option(cmdline, p, 2)) {
			p += 2;
			continue;
		}
		ld = parse_option_leading_digit(cmdline, p);
		if (ld < 0) {
			p += 2;
			continue;
		}
		switch (ld) {
		case 0:
		case 1:
			return BIT(0);
		case 2:
			return BIT(0) | BIT(1);
		case 3:
			return BIT(0) | BIT(1) | BIT(2);
		case 4:
			return BIT(0) | BIT(1) | BIT(2) | BIT(3);
		default:
			break;
		}
		p += 2;
	}

	/* Rule 1 (-s 0) and rules 2..4: *-s N* */
	p = cmdline;
	while ((p = strstr(p, "-s"))) {
		if (!is_short_option(cmdline, p, 2)) {
			p += 2;
			continue;
		}
		ld = parse_option_leading_digit(cmdline, p);
		if (ld < 0) {
			p += 2;
			continue;
		}
		if (ld >= 0 && ld <= 3)
			return BIT(ld);
		p += 2;
	}

	/* Rule 1: -c */
	p = cmdline;
	while ((p = strstr(p, "-c"))) {
		if (!is_short_option(cmdline, p, 2)) {
			p += 2;
			continue;
		}
		p += 2;
		if (*p == '\0' || *p == ' ' || *p == '\t')
			return BIT(0);
	}

	return -EINVAL;
}

static void cmdline_nulls_to_spaces(char *buf, int len)
{
	int i;

	if (len <= 0)
		return;

	while (len > 0 && buf[len - 1] == '\0')
		len--;

	for (i = 0; i < len; i++) {
		if (buf[i] == '\0')
			buf[i] = ' ';
	}
	buf[len] = '\0';
}

static int read_task_cmdline(struct task_struct *task, char *buf, size_t buflen)
{
	struct mm_struct *mm;
	unsigned long arg_start, arg_end;
	size_t len;
	int copied;

	buf[0] = '\0';

	mm = get_task_mm(task);
	if (!mm)
		return -ESRCH;

	if (!mm->arg_end) {
		mmput(mm);
		return -EINVAL;
	}

	spin_lock(&mm->arg_lock);
	arg_start = mm->arg_start;
	arg_end = mm->arg_end;
	spin_unlock(&mm->arg_lock);

	if (arg_start >= arg_end) {
		mmput(mm);
		return -EINVAL;
	}

	len = arg_end - arg_start;
	if (len >= buflen)
		len = buflen - 1;

	copied = access_process_vm(task, arg_start, buf, len, FOLL_FORCE);
	mmput(mm);

	if (copied <= 0)
		return -EFAULT;

	cmdline_nulls_to_spaces(buf, copied);
	return 0;
}

static bool task_is_isp_app(struct task_struct *task, const char *cmdline)
{
	if (strstr(task->comm, ISP_PROCESS_NAME))
		return true;
	if (cmdline && strstr(cmdline, ISP_PROCESS_NAME))
		return true;
	return false;
}

static u64 build_cache_stamp(unsigned int count, int pid, const char *cmdline)
{
	u32 hash = jhash(cmdline, strlen(cmdline), 0);

	return ((u64)count << 32) | ((u64)hash << 16) | (u32)pid;
}

/*
 * Walk processes like ps+grep isp_app: for each candidate log pid+args,
 * keep the one with largest PID in @sel.
 */
static int query_isp_app_pid_and_args(struct isp_app_info *sel)
{
	struct task_struct *p;
	char *line = NULL;
	unsigned int count = 0;
	int ret = -ENOENT;

	sel->pid = 0;
	sel->cmdline[0] = '\0';

	line = kzalloc(ISP_CMDLINE_READ_MAX, GFP_KERNEL);
	if (!line)
		return -ENOMEM;

	rcu_read_lock();
	for_each_process(p) {
		if (p != p->group_leader || !p->mm)
			continue;

		get_task_struct(p);
		rcu_read_unlock();

		/* Do not read cmdline of current while it is inside our open() path */
		if (p == current) {
			if (task_is_isp_app(p, NULL)) {
				int pid = task_pid_nr(p);

				count++;
				if (pid > sel->pid) {
					sel->pid = pid;
					sel->cmdline[0] = '\0';
				}
			}
			put_task_struct(p);
			rcu_read_lock();
			continue;
		}

		if (read_task_cmdline(p, line, ISP_CMDLINE_READ_MAX) == 0 &&
		    task_is_isp_app(p, line)) {
			int pid = task_pid_nr(p);

			count++;
			LOG(LOG_INFO,
			    "isp_execstart: found isp_app pid=%d args='%s'\n",
			    pid, line);

			if (pid > sel->pid) {
				sel->pid = pid;
				strscpy(sel->cmdline, line, sizeof(sel->cmdline));
			}
		}

		put_task_struct(p);
		rcu_read_lock();
	}
	rcu_read_unlock();

	kfree(line);

	if (sel->pid > 0)
		ret = 0;

	return ret;
}

static void do_refresh_locked(void)
{
	struct isp_app_info sel;
	u64 stamp;
	int mask;
	int err;

	err = query_isp_app_pid_and_args(&sel);
	if (err == -ENOMEM) {
		armcb_isp_allowed_ctx_mask = 0;
		armcb_isp_app_pid = 0;
		armcb_isp_daemon_running = false;
		armcb_isp_process_cache_valid = false;
		return;
	}

	if (err == -ENOENT) {
		stamp = 0;
		if (armcb_isp_process_cache_valid &&
		    stamp == armcb_isp_process_cache_stamp)
			return;

		LOG(LOG_INFO,
		    "isp_execstart: no isp_app daemon; only isp_app opener allowed\n");
		armcb_isp_allowed_ctx_mask = 0;
		armcb_isp_app_pid = 0;
		armcb_isp_daemon_running = false;
		goto out_cache;
	}

	stamp = build_cache_stamp(1, sel.pid, sel.cmdline);
	if (armcb_isp_process_cache_valid &&
	    stamp == armcb_isp_process_cache_stamp)
		return;

	armcb_isp_app_pid = sel.pid;

	if (sel.cmdline[0] == '\0') {
		LOG(LOG_INFO,
		    "isp_execstart: isp_app pid=%d (args deferred), non-isp_app denied until reload\n",
		    sel.pid);
		armcb_isp_allowed_ctx_mask = 0;
		armcb_isp_daemon_running = false;
		goto out_cache;
	}

	mask = isp_app_args_to_mask(sel.cmdline);
	if (mask < 0) {
		LOG(LOG_ERR,
		    "isp_execstart: isp_app pid=%d args='%s' -> no rule match\n",
		    sel.pid, sel.cmdline);
		armcb_isp_allowed_ctx_mask = 0;
		armcb_isp_daemon_running = false;
		goto out_cache;
	}

	armcb_isp_daemon_running = true;
	armcb_isp_allowed_ctx_mask = (unsigned int)mask & 0xf;
	LOG(LOG_INFO,
	    "isp_execstart: isp_app pid=%d args='%s' -> allowed mask 0x%x\n",
	    sel.pid, sel.cmdline, armcb_isp_allowed_ctx_mask);

out_cache:
	armcb_isp_process_cache_stamp = stamp;
	armcb_isp_process_cache_valid = true;
}

void armcb_isp_execstart_refresh_if_needed(void)
{
	if (execstart_ignore)
		return;

	mutex_lock(&armcb_isp_execstart_lock);
	/*
	 * isp_app enumerates every /dev/video* with open+QUERYCAP during startup.
	 * Do not walk tasks / read cmdline while isp_app is in our open() path.
	 */
	if (current_opener_is_isp_app()) {
		if (!armcb_isp_process_cache_valid) {
			armcb_isp_daemon_running = false;
			armcb_isp_allowed_ctx_mask = 0;
			armcb_isp_app_pid = 0;
			armcb_isp_process_cache_stamp = 0;
			armcb_isp_process_cache_valid = true;
		}
		mutex_unlock(&armcb_isp_execstart_lock);
		return;
	}
	do_refresh_locked();
	mutex_unlock(&armcb_isp_execstart_lock);
}

bool armcb_isp_execstart_ctx_allowed(unsigned int ctx_id)
{
	unsigned int m;

	if (execstart_ignore)
		return true;

	if (ctx_id >= 4)
		return true;

	/* isp_app must open every ISP video node for VIDIOC_QUERYCAP probe */
	if (current_opener_is_isp_app())
		return true;

	mutex_lock(&armcb_isp_execstart_lock);
	if (!armcb_isp_daemon_running) {
		mutex_unlock(&armcb_isp_execstart_lock);
		return false;
	}
	m = armcb_isp_allowed_ctx_mask & 0xf;
	mutex_unlock(&armcb_isp_execstart_lock);

	return !!(m & BIT(ctx_id));
}

bool armcb_isp_execstart_multi_open_order_ok(unsigned int ctx_id)
{
	unsigned int m;
	bool daemon;

	if (execstart_ignore || ctx_id == 0 || ctx_id >= 4)
		return true;

	if (current_opener_is_isp_app())
		return true;

	mutex_lock(&armcb_isp_execstart_lock);
	daemon = armcb_isp_daemon_running;
	mutex_unlock(&armcb_isp_execstart_lock);
	if (!daemon)
		return true;

	m = armcb_isp_execstart_get_allowed_mask() & 0xf;
	if (!((m & BIT(0)) && (m & ~(BIT(0)))))
		return true;

	return atomic_read(&armcb_isp_ctx0_open_refs) > 0;
}

void armcb_isp_execstart_note_ctx0_open(void)
{
	if (!execstart_ignore)
		atomic_inc(&armcb_isp_ctx0_open_refs);
}

void armcb_isp_execstart_note_ctx0_release(void)
{
	int v;

	if (execstart_ignore)
		return;
	v = atomic_dec_return(&armcb_isp_ctx0_open_refs);
	if (unlikely(v < 0)) {
		atomic_set(&armcb_isp_ctx0_open_refs, 0);
		WARN_ONCE(1, "isp_execstart: ctx0 open ref underflow\n");
	}
}

unsigned int armcb_isp_execstart_get_allowed_mask(void)
{
	unsigned int m;

	mutex_lock(&armcb_isp_execstart_lock);
	m = armcb_isp_allowed_ctx_mask & 0xf;
	mutex_unlock(&armcb_isp_execstart_lock);
	return m;
}

void armcb_isp_execstart_force_reload(void)
{
	mutex_lock(&armcb_isp_execstart_lock);
	armcb_isp_process_cache_valid = false;
	do_refresh_locked();
	mutex_unlock(&armcb_isp_execstart_lock);
}

static int execstart_allowed_mask_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n",
			 armcb_isp_execstart_get_allowed_mask());
}

static const struct kernel_param_ops execstart_allowed_mask_ops = {
	.get = execstart_allowed_mask_get,
};

module_param_cb(execstart_allowed_mask, &execstart_allowed_mask_ops, NULL, 0444);
MODULE_PARM_DESC(execstart_allowed_mask,
		 "Read-only: allowed ctx mask from isp_app pid/args.");

static int execstart_reload_set(const char *val, const struct kernel_param *kp)
{
	int err, v;

	err = kstrtoint(val, 0, &v);
	if (err)
		return err;
	if (v)
		armcb_isp_execstart_force_reload();
	return 0;
}

static int execstart_reload_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE,
			 "write 1 to re-query isp_app pid and args\n");
}

static const struct kernel_param_ops execstart_reload_ops = {
	.set = execstart_reload_set,
	.get = execstart_reload_get,
};

module_param_cb(execstart_reload, &execstart_reload_ops, NULL, 0644);
MODULE_PARM_DESC(execstart_reload,
		 "Write 1 to invalidate cache and re-query isp_app pid/args.");

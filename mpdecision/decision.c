/******************************************************************************
 *
 * 	D E C I S I O N . C
 *
 * GENERAL DESCRIPTION
 * 	Multicore Decision Engine

 Copyright (c) 2010-2012 Qualcomm Technologies, Inc.  All Rights Reserved.
 Qualcomm Technologies Proprietary and Confidential.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <time.h>
#include <getopt.h>
#include <pthread.h>
#include <poll.h>
#include <dirent.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <private/android_filesystem_config.h>
#include <linux/un.h>
#include <cutils/uevent.h>

#include "decision.h"

#ifdef MPCTL_SERVER
extern int  mpctl_server_init(void);
extern void mpctl_server_exit(void);
extern void wait_for_mpctl_server(void);
#endif

#ifdef USE_PM2
#define SYSFS_PM_MODE "/sys/module/pm2/modes"
#else
#define SYSFS_PM_MODE "/sys/module/pm_8x60/modes"
#endif

#define __NR_sched_setaffinity  (__NR_SYSCALL_BASE+241) /* ARM arch specific */

#ifdef USE_ANDROID_LOG
#define LOG_TAG "MP-Decision"
#include "cutils/log.h"
#ifdef LOGE
#    define msg(format, ...)   LOGE(format, ## __VA_ARGS__)
#else
#    define msg(format, ...)   ALOGE(format, ## __VA_ARGS__)
#endif /* ifdef LOGE */
#else
#    define msg(format, ...)   printf(format, ## __VA_ARGS__)
#endif

#define dbgmsg(format, ...) \
	if (debug_output) \
		msg(format, ## __VA_ARGS__)

#define MAX_BUF (30)
#define MAX_PATH (256)
#define MAX_CPUS (4) /* Currently supporting up to 4 core systems */

#define RQ_DEPTH "/sys/devices/system/cpu/cpu0/rq-stats/run_queue_avg"
#define RQ_POLL_MS "/sys/devices/system/cpu/cpu0/rq-stats/run_queue_poll_ms"
#define DEF_TIMER_MS "/sys/devices/system/cpu/cpu0/rq-stats/def_timer_ms"
#define CPU_LOAD_NORMALIZED "/sys/devices/system/cpu/cpu0/rq-stats/cpu_normalized_load"
#define CPU_SYSFS "/sys/devices/system/cpu"
#define CPU_BOOST_MS 500
#define MAX_UTIL_WNDW 20


static int debug_output = 0;

/* Online/offline status of cores */
static enum CORE_STATUS core_status[MAX_CPUS];

/* Default average active-threads for wakeup threshold per-core */
static const float DEF_Nw[MAX_CPUS] = {0, 1.9, 2.7, 3.5};
/* Default average active-threads for sleep threshold per-core */
static const float DEF_Ns[MAX_CPUS] = {0, 1.1, 2.1, 3.1};
/* Default average wakeup time threshold per-core (in ms) */
static const int DEF_Tw[MAX_CPUS] = {0, 140, 90, 90};
/* Default average sleep time threshold per-core (in ms) */
static const int DEF_Ts[MAX_CPUS] = {0, 190, 240, 240};
/* Default utilization threshold per-core (%).
 * Values follow the factory binary: it logs these at startup on this device. */
static const int DEF_util_high_and[MAX_CPUS] = {0, 0, 35, 0};
static const int DEF_util_high_or[MAX_CPUS] = {0, 75, 400, 400};
static const int DEF_util_low_and[MAX_CPUS] = {0, 30, 0, 0};
static const int DEF_util_low_or[MAX_CPUS] = {0, 20, 0, 0};
static const int DEF_max_up_wndw_size = 3;
static const int DEF_max_dw_wndw_size = 5;

static int num_cpus;
static float Nw_cpus[MAX_CPUS];
static float Ns_cpus[MAX_CPUS];
static unsigned int Tw_cpus[MAX_CPUS];
static unsigned int Ts_cpus[MAX_CPUS];
static int util_high_and[MAX_CPUS];
static int util_high_or[MAX_CPUS];
static int util_low_and[MAX_CPUS];
static int util_low_or[MAX_CPUS];
static int max_up_wndw_size;
static int max_dw_wndw_size;
static unsigned int poll_ms = 9;
static unsigned int decision_ms = 50;
static int stall_decision_external = 0;
static int def_timer_fd = 0;
static unsigned int boost_time = 0;

static int control_sleep_modes = 0;
static int adjust_average = 0;
static int standalone_pc_status[MAX_CPUS];
static int pc_status_core_0;

static unsigned int cpu_avg_load_window[MAX_UTIL_WNDW]={0};
static int cpu_load_index=0;
static int valid_in_wndw_up = 0;
static int valid_in_wndw_dw = 0;

static pthread_t mp_decision, hotplug_thread;

/* hotplug_mutex protects
   core_status[], stall_decision_external, and thermal_condition */
static pthread_mutex_t hotplug_mutex = PTHREAD_MUTEX_INITIALIZER;

/* bitmask for thermal_condition of hotplug cores, core 0 is ignored
 * ASSUMPTION is made here num_cpus <= 32 */
static uint32_t thermal_condition;

/* macro to check if cpu in thermal condition */
#define THERMAL_COND_CPU(cpu)   ((thermal_condition) & (1<<(cpu)))

/* bitmask for all hotplug cores, initialized in all_cores_thermal_condition */
static uint32_t hotplug_core_mask;

static int get_cpu_status(int cpu);
static int __act_on_decision(int cpu, unsigned int status);

/**************************
 * EXTERNALIZED FUNCTIONS *
 **************************/
/* Disable mp-decision hook for performance benchmarks */
int disable_decision_engine(void)
{
	if (stall_decision_external)
		return -1;

	pthread_mutex_lock(&hotplug_mutex);
	if (thermal_condition) {
		msg("%s: failed, system is in thermal condition", __func__);
		pthread_mutex_unlock(&hotplug_mutex);
		return -1;
	}

	stall_decision_external = 1;
	pthread_mutex_unlock(&hotplug_mutex);

	return 0;
}

/* Renable mp-decision, called after 'disable_decision_engine'
   for performance benchmarks */
int enable_decision_engine(void)
{
	int cpu = 0;

	if (!stall_decision_external)
		return -1;

	pthread_mutex_lock(&hotplug_mutex);
	for(cpu = 1; cpu < num_cpus; cpu++)
		core_status[cpu] = get_cpu_status(cpu);
	stall_decision_external = 0;
	pthread_mutex_unlock(&hotplug_mutex);
	return 0;
}

/* External hook for requesting performance boost */
int core_boost(int tasks)
{
	int cpu = 0;

	boost_time = CPU_BOOST_MS;

	for (cpu = 1; cpu < num_cpus; cpu++) {
		if (core_status[cpu] != CORE_UP) {
			pthread_mutex_lock(&hotplug_mutex);
			__act_on_decision(cpu, CORE_UP);
			pthread_mutex_unlock(&hotplug_mutex);
		}
	}

	return 0;
}

/*******************
 * LOCAL FUNCTIONS *
 *******************/

/* Check all secondary cores are in thermal_condition */
static int all_cores_thermal_condition(void)
{
	static int hotplug_mask_inited;
	int cpu;

	if (!hotplug_mask_inited) {
		for (cpu = 1; cpu < num_cpus; cpu++)
			hotplug_core_mask |= (1 << cpu);
		hotplug_mask_inited = 1;
	}
	return ((thermal_condition & hotplug_core_mask) == hotplug_core_mask);
}

/* Read float value from file
   * file - file name
   * val - output float pointer
   Return value - 0 if successful, -errno if failed */
static int read_file_float(char *file, float *val)
{
	int ret = -ENODEV;
	int fd = 0;
	char buf[MAX_BUF] = {0};

	fd = open(file, O_RDONLY);
	if (fd > 0) {
		/* ensure NULL terminated, sizeof(buf) - 1 */
		if (read(fd, buf, sizeof(buf) - 1) == -1)
			ret = -EINVAL;
		else {
			*val = strtof(buf, NULL);
			ret = 0;
		}
		close(fd);
	}

	return ret;
}

/* Write uint32 value from file
   * file - file name
   * val - uint32 value to write
   Return value - 0 if successful, -errno if failed */
static int write_file_uint32(char *file, unsigned int val)
{
	int ret = -ENODEV;
	int fd = 0;
	char buf[MAX_BUF] = {0};

	snprintf(buf, MAX_BUF, "%u\n", val);

	fd = open(file, O_WRONLY);
	if (fd > 0) {
		if (write(fd, buf, sizeof(buf)) == -1)
			ret = -EINVAL;
		else
			ret = 0;
		close(fd);
	}

	return ret;
}

/* Read CPU stats from sysfs nodes and compute utilization
   * hotplug_avg_load_up - computed integer value
   * hotplug_avg_load_dw - computed integer value
   * num_online - current number of online CPUs
   Return value - 0 if successful, -errno if failed */
static int get_cpu_utils(int *hotplug_avg_load_up,
				   int *hotplug_avg_load_dw,
				   int num_online)
{
	int ret = -ENODEV;
	int fd = 0;
	char buf[MAX_BUF] = {0};
	int i,j;
	int count = 0;
	int max_wndw;
	int val[MAX_CPUS] = {0};

	*hotplug_avg_load_up = 0;
	*hotplug_avg_load_dw = 0;

	if (valid_in_wndw_up < max_up_wndw_size) {
		valid_in_wndw_up++;
	}
	if (valid_in_wndw_dw < max_dw_wndw_size) {
		valid_in_wndw_dw++;
	}

	cpu_avg_load_window[cpu_load_index] = 0;

	/* Get the total normalized cpu_util */
	fd = open(CPU_LOAD_NORMALIZED, O_RDONLY);
	if (fd > 0) {
		/* ensure NULL terminated, sizeof(buf) - 1 */
		if (read(fd, buf, sizeof(buf) - 1) == -1)
			ret = -EINVAL;
		else {
			cpu_avg_load_window[cpu_load_index] = strtol(buf, NULL, 0);
			ret = 0;
		}
		close(fd);
	}

	/* compute average load across 'up' sampling periods */
	for (i = 0, j = cpu_load_index; i < valid_in_wndw_up; i++, j--) {
		*hotplug_avg_load_up += cpu_avg_load_window[j];
		if (j == 0)
			j = MAX_UTIL_WNDW;
	}
	/* compute avg loads across the whole 'up' window */
	*hotplug_avg_load_up  = *hotplug_avg_load_up/valid_in_wndw_up;

	/* compute average load across 'dw' sampling periods */
	for (i = 0, j = cpu_load_index; i < valid_in_wndw_dw; i++, j--) {
		*hotplug_avg_load_dw += cpu_avg_load_window[j];
		if (j == 0)
			j = MAX_UTIL_WNDW;
	}
	/* compute avg loads across the whole 'dw' window */
	*hotplug_avg_load_dw  = *hotplug_avg_load_dw/valid_in_wndw_dw;

	/* return to first element if we're at the circular buffer's end */
	if (++cpu_load_index == MAX_UTIL_WNDW)
		cpu_load_index = 0;

	return ret;
}

/* Read MSM power mode status from sysfs node
   * cpu - core id
   * dir - name of power mode parameter, e.g. "power_collapse"
   * filename - name of sysfs file, e.g. "idle_enabled"
   Return value - 0/1 for disabled/enabled, -1 if error */
static int get_power_mode(int cpu, char *dir, char *filename)
{
	int fd;
	char str[MAX_BUF * 3] = {0};
	char buf[MAX_BUF] = {0};
	int ret = -1;

	snprintf(str, MAX_BUF * 3, "%s/cpu%d/%s/%s", SYSFS_PM_MODE, cpu, dir, filename);
	fd = open(str, O_RDONLY);
	if (fd > 0) {
		/* ensure NULL terminated, sizeof(buf) - 1 */
		if (read(fd, buf, sizeof(buf) - 1) != -1)
			ret = atoi(buf);
		close(fd);
	}

	return ret;
}

/* Update MSM power mode status to sysfs node
   * cpu - core id
   * dir - name of power mode parameter, e.g. "power_collapse"
   * filename - name of sysfs file, e.g. "idle_enabled"
   * enable - 0/1 for disabled/enabled
   Return value - 0 if successful, -errno if error */
static int set_power_mode(int cpu, char *dir, char *filename, int enable)
{
	int ret = 0;
	int fd;
	char str[MAX_BUF *3] = {0};
	char val[MAX_BUF] = {0};

	snprintf(str, MAX_BUF * 3, "%s/cpu%d/%s/%s", SYSFS_PM_MODE, cpu, dir, filename);
	snprintf(val, MAX_BUF, "%d\n", enable);

	fd = open(str, O_RDWR);
	if (fd > 0) {
		if (write(fd, val, sizeof(val)) == -1)
			ret = -EINVAL;
		close(fd);
	}
	else
		ret = -ENODEV;

	return ret;
}

/* Print current power modes values relevant to MPdecision */
static void print_power_modes()
{
	int cpu = 0;

	if (control_sleep_modes == 0)
		return;

	dbgmsg("Current power modes:\n");

	/* Only Core 0 has power collapse idle node */
	dbgmsg("Core0 Standalone Power Collapse state: %d, Power Collapse state: %d\n",
	       get_power_mode(0, "standalone_power_collapse", "idle_enabled"),
	       get_power_mode(0, "power_collapse", "idle_enabled"));

	for (cpu = 1; cpu < num_cpus; cpu++) {
		dbgmsg("Core%d Standalone Power Collapse state: %d\n",
			cpu, get_power_mode(cpu, "standalone_power_collapse", "idle_enabled"));
	}
}

/* Save current power modes from sysfs.
   Called before updating power modes in load situation */
static void save_power_modes()
{
	int cpu = 0;

	if (control_sleep_modes == 0)
		return;

	dbgmsg("Saving power modes:\n");

	/* Only Core 0 has power collapse idle node */
	pc_status_core_0 = get_power_mode(0, "power_collapse", "idle_enabled");
	for (cpu = 0; cpu < num_cpus; cpu++) {
		standalone_pc_status[cpu] =
			get_power_mode(cpu, "standalone_power_collapse", "idle_enabled");
	}

	print_power_modes();
}

/* Restore saved power modes, writing to sysfs.
   Should only be called after save_power_modes().
   Called upon returning to one-core-up system state */
static void restore_power_modes()
{
	int cpu = 0;

	if (control_sleep_modes == 0)
		return;

	dbgmsg("Restoring power modes\n");
	set_power_mode(cpu, "power_collapse", "idle_enabled", pc_status_core_0);
	for (cpu = 0; cpu < num_cpus; cpu++) {
		set_power_mode(cpu, "standalone_power_collapse", "idle_enabled", standalone_pc_status[cpu]);
	}

	print_power_modes();
}

/* Update CPU hotplug 'online' sysfs node
   * cpu - core id
   * on - 0/1 for offline/online
   Return value: 0 if successful, -errno if error */
static int __cpu_up(int cpu, int on)
{
	int ret = 0;
	int fd;
	char str[MAX_BUF * 3] = {0};
	char val[MAX_BUF] = {0};

	snprintf(str, MAX_BUF * 3, "/sys/devices/system/cpu/cpu%d/online", cpu);
	snprintf(val, MAX_BUF, "%d\n", (on ? 1 : 0));

	fd = open(str, O_RDWR);
	if (fd > 0) {
		if (write(fd, val, sizeof(val)) == -1)
			ret = -EINVAL;
		close(fd);
	}
	else
		ret = -ENODEV;

	if (ret) {
		msg("Error %d setting %s online status to %s", errno, str, val);
	} else {
		core_status[cpu] = on;
	}
	return ret;
}

/* Utility function to read cpu
   * cpu - core id
   * on - 0/1 for offline/online
   Return value: 0/1 core status, -errno if error */
static int get_cpu_status(int cpu)
{
	int status = CORE_DOWN;
	int fd;
	char str[MAX_BUF * 3] = {0};
	char buf[MAX_BUF] = {0};

	snprintf(str, MAX_BUF * 3, "/sys/devices/system/cpu/cpu%d/online", cpu);
	fd = open(str, O_RDONLY);
	if (fd > 0) {
		/* ensure NULL terminated, sizeof(buf) - 1 */
		if (read(fd, buf, sizeof(buf) - 1) == -1)
			buf[0] = '0';
		buf[1] = '\0';
		status = atoi(buf) ? CORE_UP: CORE_DOWN;
		close(fd);
	} else {
		msg("Error %d getting cpu %d online status at %s", errno, cpu, str);
	}

	return status;
}

/* Get current number of online CPUs
   Return value: number of online CPUs */
static inline int __get_num_online_cpus()
{
	int online = 0;
	int cpu = 0;

	for (cpu = 0; cpu < num_cpus; cpu++)
		if (core_status[cpu] == CORE_UP)
			online++;

	return online;
}

/* find id of the last online secondary cpu, 0 if none */
static inline int __last_online_hotplug_cpu()
{
	int cpu;

	for (cpu = num_cpus - 1; cpu >= 1; cpu--)
		if (core_status[cpu] == CORE_UP)
			return cpu;

	return 0;
}

/* Check if MPdecision is stalled
   Return value: true/false */
static inline int decision_engine_stalled(void)
{
	return stall_decision_external;
}

/* Updates the CPU online status, with side-effect of controlling sleep modes
   (if the option is enabled). In leaving one-core-up system state (to deal with
   system load), power modes should be saved and idle power collapse modes
   disabled. On returning to one-core-up state, power modes should be restored.
   Must called with hotplug_mutex held.
   * cpu - core id
   * status - 0/1 for offline/online
   Return value - 0 if successful, -errno if error */
static int __cpu_action_control_sleep(int cpu, unsigned int status)
{
	int ret = 0;
	static int sleep_modes_saved = 0;

	if (status == core_status[cpu]) {
		dbgmsg("No action required to change core status(%d).", status);
		return 0;
	}

	dbgmsg("Action to bring core %d %s\n", cpu, (status == CORE_UP) ? "online" : "offline");

	switch (status) {
	case CORE_UP:
		if (control_sleep_modes == 1) {
			int err = 0;
			int num_online = __get_num_online_cpus();

			if (num_online == 1 && sleep_modes_saved == 0) {
				/* Excluding core0 */
				/* Remember the individual core independent power collapse setting for cores */
				dbgmsg("Saving power modes before CORE_UP\n");
				save_power_modes();
				sleep_modes_saved = 1;
				/* Disable individual core independent power collapse for core0 */
				err += set_power_mode(0, "standalone_power_collapse", "idle_enabled", 0);
				err += set_power_mode(0, "power_collapse", "idle_enabled", 0);
				if (err)
					msg("Error setting a sleep mode on core 0 - %d\n", err);
			}
			/* Disable individual core independent power collapse */
			err += set_power_mode(cpu, "standalone_power_collapse", "idle_enabled", 0);
			if (err)
				msg("Error setting a sleep mode for secondary cores - %d\n", err);
		}

		/* Power up the core */
		ret = __cpu_up(cpu, CORE_UP);
		break;
	case CORE_DOWN:
		/* Power down the core */
		ret = __cpu_up(cpu, CORE_DOWN);

		/* Restore the original PC states */
		if (control_sleep_modes == 1)  {
			int num_online = __get_num_online_cpus();

			if (num_online == 1 && sleep_modes_saved == 1) {
				/* core1 was just offlined */
				dbgmsg("Restoring power modes after CORE_DOWN\n");
				restore_power_modes();
				sleep_modes_saved = 0;
			}
		}
		break;
	default:
		break;
	}

	if (ret) {
		msg("Error(%d) changing core %d status to %s\n", ret, cpu,
				status ? "online" : "offline");
	} else {
		dbgmsg("Core %d is now %s", cpu,
		       (status == CORE_UP ? "online" : "offline"));
	}

	return ret;
}

/* Acts on MPdecision algorithm to update core status.
   Decision may be ignored if externally stalled or
   system in thermal condition.
   * cpu - core id
   * status - 0/1 for offline/online
   Return value - 0 if successful, -errno if error */
static int __act_on_decision(int cpu, unsigned int status)
{
       if (decision_engine_stalled()) {
               msg("MP decision to set core%d status to %d ignored\n", cpu, status);
               return -ENODEV;
       }

       if (THERMAL_COND_CPU(cpu)) {
	       msg("CPU%d in thermal condition, request to set core status "
		   "to %d ignored\n", cpu, status);
	       return -ENODEV;
       }

       return __cpu_action_control_sleep(cpu, status);
}

/* Wait for unstalling of decision engine
   (currently no wait implemented, busy wait)
   Return value - 0 if no wait required, 1 if decision stalled */
static int wait_for_cpu_online(void)
{
	if (!decision_engine_stalled())
		return 0;

	return 1;
}

/* Do deferrable sleep (allowing for power collapse)
   * sleep_ms - number of millisecs to sleep
   Return value - length of time slept */
static int deferrable_sleep(int sleep_ms)
{
	int err = 0;
	int time_ms = 0;
	struct pollfd fds;
	char buf[20] = {0};

	write_file_uint32(DEF_TIMER_MS, sleep_ms);

	while (1) {
		fds.fd = def_timer_fd;
		fds.events = POLLERR|POLLPRI;
		fds.revents = 0;

		err = poll(&fds, 1, -1);
		if (err == -1) {
			msg("Error waiting for timer\n");
			close(fds.fd);
			return 0;
		}

		/* ensure NULL terminated, sizeof(buf) - 1 */
		if (read(def_timer_fd, buf, sizeof(buf) - 1) != -1) {
			lseek(def_timer_fd, 0, SEEK_SET);

			time_ms = atoi(buf);
		}
		if (time_ms)
			break;
	}

	return time_ms;
}

/* Main mpdecision algorithm thread */
static void *do_mp_decision(void *data)
{
	float rq_depth = 0;
	float time = 0;
	float total_time_up[MAX_CPUS] = {0};
	float total_time_down[MAX_CPUS] = {0};
	float Nw = 0.0;
	float Ns = 0.0;
	int Tw = 0;
	int Ts = 0;
	int num_online = 1;
	int cpus_reqd = 1;
	int cpu = 0;
	int cpus_up = 0;
	int cpus_down = 0;
	int reset_reqd = 0;
	int cpus_unavail = 0;
	int core_idx = 0;
	enum CORE_STATUS curr_online_status = CORE_DOWN;
	int hotplug_avg_load_up = 0;
	int hotplug_avg_load_dw = 0;
	int condition1 = 0;
	int condition2 = 0;
	int cond1_util_and = 0;
	int cond2_util_or = 0;

	while (1) {

		time = deferrable_sleep(decision_ms);

		/* Block if hotplugged externally. */
		/* We cannot account for the hotplugged time correctly, reset and continue */
		if (wait_for_cpu_online()) {
			memset(total_time_up, 0, sizeof(float) * MAX_CPUS);
			memset(total_time_down, 0, sizeof(float) * MAX_CPUS);
			dbgmsg("MP-Decision was stalled. Will reset and continue.\n");
			continue;
		}

		if (all_cores_thermal_condition()) {
			memset(total_time_up, 0, sizeof(float) * MAX_CPUS);
			memset(total_time_down, 0, sizeof(float) * MAX_CPUS);
			dbgmsg("MP-Decision is disabled in thermal condition. Will reset and continue.\n");
			continue;
		}

		if (read_file_float(RQ_DEPTH, &rq_depth) < 0) {
			msg("Error reading run queue depth.\n");
			continue;
		}

		if (rq_depth < 0) {
			msg("Invalid run queue depth :%f\n", rq_depth);
			continue;
		}

		pthread_mutex_lock(&hotplug_mutex);

		/*
		 * Check our state from last cycle was valid, if somebody has onlined/offlined cores
		 * outside mpdecision, then our accounting so far is now invalid. Reset and continue.
		 * We will use the same cpus_req, since its as good as resetting to 1 or all num_cpus.
		 */
		reset_reqd = 0;
		for (cpu = 1; cpu < num_cpus; cpu++) {
			curr_online_status = (enum CORE_STATUS) get_cpu_status(cpu);
			if (curr_online_status != core_status[cpu] &&
			    curr_online_status != CORE_UP) {
				__cpu_up(cpu, CORE_UP);
				reset_reqd = 1;
				dbgmsg("Resetting core %d status to online.", cpu);
			} else if (curr_online_status != core_status[cpu] &&
				   curr_online_status != CORE_DOWN) {
				__cpu_up(cpu, CORE_DOWN);
				reset_reqd = 1;
				dbgmsg("Resetting core %d status to offline.", cpu);
			}
		}
		if (reset_reqd) {
			memset(total_time_up, 0, sizeof(float) * MAX_CPUS);
			memset(total_time_down, 0, sizeof(float) * MAX_CPUS);
			goto unlock;
		}

		num_online = __get_num_online_cpus();

		/*get cpu utilization */
		if(get_cpu_utils(&hotplug_avg_load_up, &hotplug_avg_load_dw, num_online) < 0) {
			msg("Error reading cpu load.\n");
			continue;
		}

		/*
		 * If the timer has been deferred, set runQ to zero
		 */
		if (adjust_average == 1 && time > decision_ms) {
			dbgmsg("Runqueue depth :%f time :%f (deferred)\n", rq_depth, time);
			rq_depth = 0;
		}
		dbgmsg("Runqueue depth :%f load_up :%d load_dw :%d time :%f online :%d boost :%d ms\n",
			rq_depth, hotplug_avg_load_up, hotplug_avg_load_dw, time, num_online, boost_time);

		cpus_up = 0;
		cpus_down = 0;

		cpus_unavail= 0;
		for (cpu = 1; cpu < num_cpus; cpu++) {
			if (THERMAL_COND_CPU(cpu)) {
				cpus_unavail++;
				continue;
			}

			/* Parameters based on core index instead of cpu index.
			   e.g. CPU 2 may be 1st available secondary core,
			   if CPU 1 unavailable */
			core_idx = cpu - cpus_unavail;

			Nw = Nw_cpus[core_idx];
			Tw = Tw_cpus[core_idx];
			Ns = Ns_cpus[core_idx];
			Ts = Ts_cpus[core_idx];

			/* Check the runq and update/reset the time counters*/
			if (core_status[cpu] == CORE_DOWN) {
				/* 'cpu' is offline */
				if (rq_depth >= Nw)
					total_time_up[core_idx] += time;
				else
					total_time_up[core_idx] = 0;
				total_time_down[core_idx] = 0;
			} else {
				/* 'cpu' is online */
				if (rq_depth <= Ns)
					total_time_down[core_idx] += time;
				else
					total_time_down[core_idx] = 0;
				total_time_up[core_idx] = 0;
			}

			/* Check if CPU core_idx needs to be added. No need
			 * to add a core that is already online (CORE_UP)*/
			if (core_status[cpu] == CORE_DOWN) {
				cond1_util_and =  util_high_and[num_online]; /* NEW */
				if (core_idx == num_online) {
					/* if it is considering to bring online just one additional core */
					cond2_util_or = util_high_or[core_idx];
				} else {
					/*
					 * if it is adding multiple cores at once, then
					 * - the system must be 100% busy
					 * - cannot add multiple core by using the util_or condition only
					 */
					cond2_util_or = INT_MAX;
				}

				/* Verify the conditions to bring the core online */
				condition1 = ((total_time_up[core_idx] >= Tw) && (hotplug_avg_load_up >= cond1_util_and));
				condition2 = (hotplug_avg_load_up >= cond2_util_or);

				if (condition1 || condition2)
					cpus_up = core_idx;
			}

			/* check if CPU core_idx needs to be removed */
			if ((core_status[cpu] == CORE_UP) && !cpus_down) {
				condition1 = ((total_time_down[core_idx] >= Ts) && (hotplug_avg_load_dw < util_low_and[core_idx]));
				condition2 = (hotplug_avg_load_dw < util_low_or[core_idx]);

				if(condition1 || condition2)
					cpus_down = core_idx;
			}

			dbgmsg("cpu%d: core_idx: %d uptime: %f downtime %f\n", cpu, core_idx, total_time_up[core_idx], total_time_down[core_idx]);
			dbgmsg("cpu%d: core_idx: %d Nw: %f, Tw :%d, Ns: %f, Ts: %d\n", cpu, core_idx, Nw, Tw, Ns, Ts);
		}

		dbgmsg("cpus_up: %d cpus_down: %d thermal_mask: 0x%x\n",
		       cpus_up, cpus_down, thermal_condition);

		if (!cpus_up && !cpus_down)
			goto unlock;

		if (cpus_up)
			cpus_reqd = cpus_up + 1;
		else if (cpus_down)
			cpus_reqd = cpus_down;
		else
			cpus_reqd = num_online;

		if (boost_time > 0) {
			if (boost_time < decision_ms)
				boost_time = 0;
			else
				boost_time -= decision_ms;
		}

		/* Change core state if there are available cpus */
		if (cpus_reqd != num_online) {
			dbgmsg("num online cores: %d reqd : %d available : %d\n",
			       num_online, cpus_reqd, num_cpus - cpus_unavail);
			if (cpus_reqd < num_online) {
				for (cpu = __last_online_hotplug_cpu();
				     cpu >= 1 && cpus_reqd < num_online; cpu--) {
					if (THERMAL_COND_CPU(cpu))
						continue;
					core_idx = cpu - cpus_unavail;
					dbgmsg("DOWN cpu:%d core_idx:%d Ns:%f Ts:%d rq:%f seq:%f\n", cpu, core_idx,
						Ns_cpus[core_idx], Ts_cpus[core_idx], rq_depth, total_time_down[core_idx]);
					dbgmsg("load_dw: %d u_low_and: %d, u_low_or:%d\n", hotplug_avg_load_dw, util_low_and[core_idx], util_low_or[core_idx]);
					__act_on_decision(cpu, CORE_DOWN);
					total_time_down[core_idx] = 0;
					num_online = __get_num_online_cpus();
				}
			} else if (num_cpus - cpus_unavail > num_online) {
				for (cpu = 1; cpu < num_cpus && cpus_reqd > num_online; cpu++) {
					if (THERMAL_COND_CPU(cpu))
						continue;
					core_idx = cpu - cpus_unavail;
					dbgmsg("UP cpu:%d core_idx:%d Nw:%f Tw:%d rq:%f seq:%f\n", cpu, core_idx,
						Nw_cpus[core_idx], Tw_cpus[core_idx], rq_depth, total_time_up[core_idx]);
					dbgmsg("load_up: %d u_and: %d, u_or: %d\n", hotplug_avg_load_up, cond1_util_and, cond2_util_or);
					__act_on_decision(cpu, CORE_UP);
					total_time_up[core_idx] = 0;
					num_online = __get_num_online_cpus();
				}
			}
			/*reset the number of valid samples in the cpu
			  utilization window upon a cpu state change*/
			valid_in_wndw_up = 0;
			valid_in_wndw_dw = 0;
		}

unlock:
		pthread_mutex_unlock(&hotplug_mutex);
	}

	return NULL;
}

/* hotplug thread - listens for kernel hotplug events */
static void *do_hotplug(void *data)
{
	int err = 0;
	int cpu = 1;
	struct pollfd fds;
	char buf[1024] = {0};
	char cpu_online[MAX_BUF * 2] = {0};
	char cpu_offline[MAX_BUF * 2] = {0};
	char temp[MAX_BUF * 2] = {0};
	unsigned int online;
	int recv_bytes = 0;

	/* Looking for online@/devices/system/cpu/cpuX or
	 * offline@/devices/system/cpu/cpuX */
	snprintf(cpu_online, MAX_BUF * 2, "online@/devices/system/cpu/cpu" );
	snprintf(cpu_offline, MAX_BUF * 2, "offline@/devices/system/cpu/cpu");

	fds.events = POLLIN;
	fds.fd = uevent_open_socket(64*1024, true);
	if (fds.fd == -1) {
		msg("Error opening socket for hotplug uevent.\n");
		return NULL;
	}

	while (1) {

		err = poll(&fds, 1, -1);
		if (err == -1) {
			msg("Error in hotplug CPU[%d] poll.\n", cpu);
			break;
		}

		recv_bytes = uevent_kernel_multicast_recv(fds.fd, buf, sizeof(buf));
		if (recv_bytes == -1)
			continue;
		if (recv_bytes >= 1024)
			buf[1023] = '\0';
		else
			buf[recv_bytes] = '\0';

		dbgmsg("Hotplug read: %s\n", buf);

		if (strstr(buf, cpu_online) != NULL)
			online = 1;
		else if (strstr(buf, cpu_offline) != NULL)
			online = 0;
		else
			continue;

		if (online)
			err = sscanf(buf, "online@/devices/system/cpu/cpu%d", &cpu);
		else
			err = sscanf(buf, "offline@/devices/system/cpu/cpu%d", &cpu);

		pthread_mutex_lock(&hotplug_mutex);

		/* Stall the algorithm based on external input */
		if (stall_decision_external && !thermal_condition)
			goto unlock;

		if (err == 1 && cpu < num_cpus) {
			/* Disallow cores to be online in a thermal situation */
			if (THERMAL_COND_CPU(cpu) && online) {
				msg("Core%d in thermal condition, bring core "
				    "back down.", cpu);
				__cpu_up(cpu, CORE_DOWN);
			}
		}
unlock:
		pthread_mutex_unlock(&hotplug_mutex);
	}

	return NULL;
}

/* Parse commandline arguments
   * argc - number of arguments
   * argv - argument array */
static void parse_args(int argc, char *argv[])
{
	float Nw, Ns;
	int Tw, Ts;
	int cpu;
	int util;

	while (1) {
		int idx = 0;
		int c = 0;
		static struct option long_options[] = {
			{"Nw", required_argument, NULL, 1},
			{"Tw", required_argument, NULL, 2},
			{"Ns", required_argument, NULL, 3},
			{"Ts", required_argument, NULL, 4},
			{"poll_ms", required_argument, NULL, 5},
			{"decision_ms", required_argument, NULL, 6},
			{"no_sleep", no_argument, NULL, 7},
			{"avg_comp", no_argument, NULL, 8},
			{"debug", no_argument, NULL, 9},
			{"util_h_and", required_argument, NULL, 10},
			{"util_h_or", required_argument, NULL, 11},
			{"util_l_and", required_argument, NULL, 12},
			{"util_l_or", required_argument, NULL, 13},
			{NULL, 0, 0, 0},
		};

		c = getopt_long_only(argc, argv, "", long_options, &idx);
		if (c == -1)
			break;

		switch (c) {
		case 1:
			sscanf(optarg, "%d:%f", &cpu, &Nw);
			if ((cpu > 0 && cpu < num_cpus) && (Nw > 0 && Nw < 4))
				Nw_cpus[cpu] = Nw;
			break;
		case 2:
			sscanf(optarg, "%d:%d", &cpu, &Tw);
			if ((cpu > 0 && cpu < num_cpus) && (Tw > 0 && Tw < 1000000))
				Tw_cpus[cpu] = Tw;
			break;
		case 3:
			sscanf(optarg, "%d:%f", &cpu, &Ns);
			if ((cpu > 0 && cpu < num_cpus) && (Ns > 0 && Ns < 4))
				Ns_cpus[cpu] = Ns;
			break;
		case 4:
			sscanf(optarg, "%d:%d", &cpu, &Ts);
			if ((cpu > 0 && cpu < num_cpus) && (Ts > 0 && Ts < 1000000))
				Ts_cpus[cpu] = Ts;
			break;
		case 5:
			poll_ms = atoi(optarg);
			break;
		case 6:
			decision_ms = atoi(optarg);
			break;
		case 7:
			control_sleep_modes = 1;
			msg("OPTION ENABLED: Control sleep modes");
			break;
		case 8:
			adjust_average = 1;
			msg("OPTION ENABLED: Adjusting average");
			break;
		case 9:
			debug_output = 1;
			msg("OPTION ENABLED: Debug output");
			break;
		case 10:
			sscanf(optarg, "%d:%d", &cpu, &util);
			if ((cpu > 0 && cpu < num_cpus) && (util > 0 && util < (num_cpus*100)))
				util_high_and[cpu] = util;
			break;
		case 11:
			sscanf(optarg, "%d:%d", &cpu, &util);
			if ((cpu > 0 && cpu < num_cpus) && (util > 0 && util < (num_cpus*100)))
				util_high_or[cpu] = util;
			break;
		case 12:
			sscanf(optarg, "%d:%d", &cpu, &util);
			if ((cpu > 0 && cpu < num_cpus) && (util > 0 && util < (num_cpus*100)))
				util_low_and[cpu] = util;
			break;
		case 13:
			sscanf(optarg, "%d:%d", &cpu, &util);
			if ((cpu > 0 && cpu < num_cpus) && (util > 0 && util < (num_cpus*100)))
				util_low_or[cpu] = util;
			break;
		default:
		case '?':
			msg("\n MP Decision Options:\n");
			msg("--poll_ms --decision_ms --no_sleep --avg_comp --debug\n");
			msg("--Nw=[1..3]:[value] --Tw=[1..3]:[value] --Ns=[1..3]:[value] --Ts=[1..3]:[value]\n");
			msg("--util_h_and=[1..3]:[value] --util_h_or=[1..3]:[value]\n");
			msg("--util_l_and=[1..3]:[value] --util_l_or=[1..3]:[value]\n");
			exit(-1);
			break;
		}
	}

	for (cpu = 0; cpu < num_cpus; cpu++) {
		msg("Decision parameters CPU %d: Nw=%f, Tw=%d, Ns=%f, Ts=%d\n",
			cpu, Nw_cpus[cpu], Tw_cpus[cpu], Ns_cpus[cpu], Ts_cpus[cpu]);
		msg("util_h_and=%d, util_h_or=%d, util_low=%d\n",
			util_high_and[cpu], util_high_or[cpu], util_low_or[cpu]);
	}
	msg("Decision parameters: poll_ms: %d decision_ms: %d\n", poll_ms, decision_ms);
}

/* Reads from CPU_SYSFS folder to system CPUs count
   Return value - number of cpus, -1 if error */
static unsigned int get_num_cpus()
{
	DIR *tdir = NULL;
	struct dirent *tdirent = NULL;
	int ncpus = 0;
	char name[MAX_PATH] = {0};
	char cwd[MAX_PATH] = {0};
	int cpu = 0;
	char buf[50];
	struct dirent *cpu_dirent;

	if (!getcwd(cwd, sizeof(cwd)))
		return -1;

	chdir(CPU_SYSFS); /* Change dir to read the entries. Doesnt work
				 otherwise */
	tdir = opendir(CPU_SYSFS);
	if (!tdir) {
		msg("Unable to open %s\n", CPU_SYSFS);
		return -1;
	}

	while ((cpu_dirent = readdir(tdir))) {
		if (strstr(cpu_dirent->d_name, "cpu") == NULL)
			continue;
		if (!sscanf(cpu_dirent->d_name, "cpu%d", &cpu))
			continue;
		ncpus++;
	}

	closedir(tdir);
	chdir(cwd); /* Restore current working dir */

	dbgmsg("Number of cores %d\n", ncpus);

	return ncpus;
}

/* Forces all secondary cores down to deal with thermal condition
   * enable - 0/1 for thermal condition enable/disable */
static void thermal_force_coredown(int cpu, int enable)
{
	if ((enable == 0 && THERMAL_COND_CPU(cpu) == 1) ||
	    (enable == 1 && THERMAL_COND_CPU(cpu) == 0)) {
		msg("%s: invalid request for thermal", __func__);
		return;
	}

	pthread_mutex_lock(&hotplug_mutex);

	if (enable == 0) {
		msg("%s: Request received to force cpu%d down", __func__, cpu);
		__cpu_action_control_sleep(cpu, CORE_DOWN);
		thermal_condition |= (1 << cpu);
	} else {
		msg("%s: Request received to remove thermal condition on cpu%d",
		    __func__, cpu);
		thermal_condition &= ~(1 << cpu);
	}
	pthread_mutex_unlock(&hotplug_mutex);
}

static pthread_t mpdecision_server_thread;
static struct sockaddr_un addr;
static int comsoc = -1;
#define MPDECISION_SOCKET       "/dev/socket/mpdecision"
#define NUM_LISTEN_QUEUE        (2)

/* Server thread for listening to external messages
   Currently only thermald is a known client for thermal hotplug fallback */
static void *mpdecision_server(void *data)
{
	int rc, len;
	int cpu;
	int online;
	int conn_socket = 0;
	char msgbuf[MAX_BUF] = {0};

	if (comsoc < 0)
		return NULL;

	for (;;) {
		len = sizeof(struct sockaddr_un);
		conn_socket = accept(comsoc, (struct sockaddr *) &addr, &len);
		if (conn_socket == -1) {
			msg("%s: accept failed", __func__);
			continue;
		}

		memset(msgbuf, 0, MAX_BUF);
		rc = recv(conn_socket, msgbuf, MAX_BUF, 0);
		if (rc <= 0) {
			msg("%s: recv failed", __func__);
			goto close_conn;
		}
		dbgmsg("%s: received msg %s", __func__, msgbuf);

		if (rc >= MAX_BUF)
			msgbuf[MAX_BUF - 1] = '\0';
		else
			msgbuf[rc] = '\0';

		rc = sscanf(msgbuf, "hotplug %d %d", &cpu, &online);
		if (rc != 2) {
			msg("%s: msg format unexpected", __func__);
		} else if (cpu <= 0 || cpu >= num_cpus) {
			msg("%s: invalid cpu %d unexpected", __func__, cpu);
		} else if (!(online == 0 || online == 1)) {
			msg("%s: online %d unexpected", __func__, online);
		} else {
			thermal_force_coredown(cpu, online);
		}

close_conn:
		close(conn_socket);
	}

	return NULL;
}

/* Initialize server socket for listening to external messages
   Unix abstract namespace socket at MPDECISION_SOCKET
   Return value: 1 if successful, 0 otherwise */
static int mpdecision_server_init()
{
	int rc = 0, stage = 0;
	int len = 0;
	msg("MPDecision server starting");

	comsoc = socket(PF_UNIX, SOCK_STREAM, 0);
	if (comsoc < 0) {
		stage = 1;
		goto error;
	}

	memset(&addr, 0, sizeof(struct sockaddr_un));
	snprintf(addr.sun_path, UNIX_PATH_MAX, MPDECISION_SOCKET);
	addr.sun_family = AF_UNIX;

	/* Delete existing socket file if necessary */
	unlink(addr.sun_path);

	rc = bind(comsoc, (struct sockaddr *)&addr, sizeof(struct sockaddr_un));
	if (rc != 0) {
		stage = 2;
		goto error;
	}

	chown(addr.sun_path, AID_ROOT, AID_SYSTEM);
	chmod(addr.sun_path, 0660);

	rc = listen(comsoc, NUM_LISTEN_QUEUE);
	if (rc != 0) {
		stage = 3;
		goto error;
	}

	rc = pthread_create(&mpdecision_server_thread, NULL, mpdecision_server, NULL);
	if (rc != 0) {
		stage = 4;
		goto error;
	}
	return 1;

error:
	msg("Unable to create control service (stage=%d, rc=%d)", stage, rc);
	if (comsoc != -1) {
		close(comsoc);
		comsoc = -1;
	}
	return 0;
}

/* Cleanup MPDecision server resources */
static void mpdecision_server_exit()
{
	if (comsoc != -1) {
		close(comsoc);

		pthread_join(mpdecision_server_thread, NULL);
		comsoc = -1;
	}
}

/* Main entry to mpdecision
   * argc - number of command-line arguments
   * argv - command-line argument array
   Return value - 0 if successful, negative otherwise */
int main(int argc, char *argv[])
{
	int ret = 0;
	int cpu = 1;
	unsigned int cpu_set = 1; /* Run on core0 */

#ifdef MPCTL_SERVER
	mpctl_server_init();
#endif
	num_cpus = get_num_cpus();
	if (num_cpus == 1) {
#ifdef MPCTL_SERVER
		/*wait for mpctl_server thread to exit*/
		wait_for_mpctl_server();
#endif

		return ret;
	}
	mpdecision_server_init();

	ret = syscall(__NR_sched_setaffinity,0, sizeof(cpu_set), &cpu_set);
	if (ret < 0) {
		msg("Cannot set cpu affinity: %s\n", strerror(-ret));
		return ret;
	}

	if (num_cpus > MAX_CPUS) {
		msg("CPUs detected: %d, not supported. Exiting.\n", num_cpus);
		return -1;
	}

	/* Priority */
	setpriority(PRIO_PROCESS, getpid(), -20);

	/* Init MP-Decision parameters */
	for(cpu = 0; cpu < num_cpus; cpu++) {
		core_status[cpu] = CORE_UP;
		Nw_cpus[cpu] = DEF_Nw[cpu];
		Tw_cpus[cpu] = DEF_Tw[cpu];
		Ns_cpus[cpu] = DEF_Ns[cpu];
		Ts_cpus[cpu] = DEF_Ts[cpu];
		util_high_and[cpu] = DEF_util_high_and[cpu];
		util_high_or[cpu]  = DEF_util_high_or[cpu];
		util_low_and[cpu]      = DEF_util_low_and[cpu];
		util_low_or[cpu]      = DEF_util_low_or[cpu];
		max_up_wndw_size        = DEF_max_up_wndw_size;
		max_dw_wndw_size        = DEF_max_dw_wndw_size;
	}

	/* Command line overrides */
	parse_args(argc, argv);

	/* Enable kernel calculation of rq depth */
	write_file_uint32(RQ_POLL_MS, poll_ms);

	def_timer_fd = open(DEF_TIMER_MS, O_RDONLY);
	if (def_timer_fd < 0) {
		msg("Unable to open deferrable timer file\n");
		return -1;
	}

	pthread_create(&hotplug_thread, NULL, do_hotplug, (void *)NULL);

	/* Bring all the secondary cores down */
	for(cpu = 1; cpu < num_cpus; cpu++) {
		pthread_mutex_lock(&hotplug_mutex);
		if (CORE_UP == get_cpu_status(cpu)) {
			ret = __cpu_up(cpu, CORE_DOWN);
			if (ret)
				msg("Unable to bring cpu%d down on start\n", cpu);
		} else {
			core_status[cpu] = CORE_DOWN;
		}
		pthread_mutex_unlock(&hotplug_mutex);
	}

	pthread_create(&mp_decision, NULL, do_mp_decision, NULL);

	pthread_join(mp_decision, NULL);
	pthread_join(hotplug_thread, NULL);

	close(def_timer_fd);

	mpdecision_server_exit();

#ifdef MPCTL_SERVER
	mpctl_server_exit();
#endif

	return ret;
}

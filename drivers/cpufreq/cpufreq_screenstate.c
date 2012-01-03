/*
 *  linux/drivers/cpufreq/cpufreq_screenstate.c
 *
 *  Screenstate V1 1.5
 *
 *  Marco Benton marco@unixpsycho.com 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>

static bool lcd_state = 1; // FIXME: need to make this more reliable
static bool cpu_is_managed = 0;

static DEFINE_MUTEX(screenstate_mutex);

void cpufreq_set_ss_state(bool state) {
	struct cpufreq_policy *policy;

	if(cpu_is_managed) {
		policy = cpufreq_cpu_get(0);
		BUG_ON(!policy);

		if(!(lcd_state = state))
			__cpufreq_driver_target(policy, policy->min,
						CPUFREQ_RELATION_L);
		else
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);

		printk("screenstate: lcd %s\n", lcd_state ? "on" : "off");
	}
}
EXPORT_SYMBOL(cpufreq_set_ss_state);

static int cpufreq_governor_screenstate(struct cpufreq_policy *policy,
				   unsigned int event) {
	unsigned int cpu = policy->cpu;

	switch (event) {
		case CPUFREQ_GOV_START:
			if (!cpu_online(cpu)) return -EINVAL;
			if(cpu_is_managed) goto out;

			mutex_lock(&screenstate_mutex);

			cpu_is_managed = 1;
			printk("screenstate on CPU %d: initialized\n",cpu);

			mutex_unlock(&screenstate_mutex);
			break;
		case CPUFREQ_GOV_STOP:
			mutex_lock(&screenstate_mutex);

			cpu_is_managed = 0;
			printk("screenstate on CPU %d: uninitialized\n",cpu);

			mutex_unlock(&screenstate_mutex);
			break;
		case CPUFREQ_GOV_LIMITS:
			mutex_lock(&screenstate_mutex);

			printk("screenstate: policy change on cpu #%d"
				" min=%d max=%d\n", cpu, policy->min,
							policy->max);
			if(lcd_state && (policy->cur != policy->max))
				__cpufreq_driver_target(policy, policy->max,
							CPUFREQ_RELATION_H);
			if(!lcd_state && (policy->cur != policy->min))
				__cpufreq_driver_target(policy, policy->min,
							CPUFREQ_RELATION_L);

			mutex_unlock(&screenstate_mutex);
			break;
		}

out:
		return 0;
}

struct cpufreq_governor cpufreq_gov_screenstate = {
	.name		= "screenstate",
	.governor	= cpufreq_governor_screenstate,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_screenstate_init(void) {
	return cpufreq_register_governor(&cpufreq_gov_screenstate);
}

static void __exit cpufreq_gov_screenstate_exit(void) {
	cpufreq_unregister_governor(&cpufreq_gov_screenstate);
}

EXPORT_SYMBOL(cpufreq_gov_screenstate);

MODULE_AUTHOR ("marco@unixpsycho.com");
MODULE_DESCRIPTION ("CPUfreq policy governor 'screenstate'");
MODULE_LICENSE ("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCREENSTATE
fs_initcall(cpufreq_gov_screenstate_init);
#else
module_init(cpufreq_gov_screenstate_init);
#endif
module_exit(cpufreq_gov_screenstate_exit);

/*
 *  drivers/cpufreq/cpufreq_override.c
 *
 *      Marco Benton <marco@unixpsycho.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/cpu.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>

// Voltage min
#define VDD_MIN 800000

// Voltage max
#define VDD_MAX 1500000

// Max Freq count. Not the actual number of freqs
#define MAX_FREQS 35

/* ************* end of tunables ***************************************** */

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
void acpuclk_get_voltages(unsigned int acpu_freqs_vlt_tbl[]);
void acpuclk_set_voltages(unsigned int acpu_freqs_vlt_tbl[]);
#endif

unsigned int acpuclk_get_freqs(unsigned int acpu_freqs_tbl[]);
static unsigned int freq_table[MAX_FREQS];
static unsigned int nr_freqs;

#define define_one_ro(_name)            \
static struct global_attr _name =       \
__ATTR(_name, 0444, show_##_name, NULL)

#define define_one_rw(_name) \
static struct global_attr _name =       \
__ATTR(_name, 0644, show_##_name, store_##_name)

char *skip_spaces(const char *str)
{
        while (*str == ' ' || *str == '\t')
                ++str;
        return (char *)str;
}

char *find_spaces(const char *str)
{
        while (*str != ' ' && *str != '\t')
                ++str;
        return (char *)str;
}

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
static ssize_t show_vdd_max(struct kobject *k, struct attribute *a, char *buf)
{
	return sprintf(buf, "%u\n", VDD_MAX);
}

static ssize_t show_vdd_min(struct kobject *k, struct attribute *a, char *buf)
{
	return sprintf(buf, "%u\n", VDD_MIN);
}

static ssize_t show_vdd(struct kobject *k, struct attribute *a, char *buf)
{
	unsigned int i, acpu_freq_vlt_tbl[MAX_FREQS];
	char tmp[250];

	acpuclk_get_voltages(acpu_freq_vlt_tbl);

	strcpy(buf,"");

	for(i=0 ; i < nr_freqs ; ++i) {
		sprintf(tmp,"%u ",acpu_freq_vlt_tbl[i] * 1000);
		strcat(buf,tmp);
	}

	strcpy(tmp,buf);

	return sprintf(buf,"%s\n",tmp);
}

static ssize_t store_vdd(struct kobject *a, struct attribute *b,const char *buf, size_t count)
{
	unsigned int i = 0, acpu_freq_vlt_tbl[MAX_FREQS];
	unsigned int *c = acpu_freq_vlt_tbl;
	const char *wp = buf;

	for(i = 0; i < nr_freqs; i++) {
		wp=skip_spaces(wp);
		sscanf(wp,"%u",&c[i]);
		if(c[i] < VDD_MIN || c[i] > VDD_MAX) break;
		c[i] /= 1000;
		c[i] = DIV_ROUND_CLOSEST(c[i] , 25) * 25;
		wp=find_spaces(wp);
  	}

	if(i != nr_freqs)
		printk("override: store_vdd invalid\n");
	else {
		acpuclk_set_voltages(acpu_freq_vlt_tbl);

		printk("override: set vdd %s\n",buf);
	}

	return count;
}
#endif

static ssize_t show_vdd_freqs(struct kobject *k, struct attribute *a, char *buf)
{
	unsigned int i;
	char tmp[250];

	*buf='\0';
	*tmp='\0';

	for(i=0 ; i < nr_freqs ; ++i) {
		sprintf(tmp,"%u ", freq_table[i]);
		strcat(buf,tmp);
	}

	strcpy(tmp,buf);

	return sprintf(buf,"%s\n",tmp);
}

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
define_one_ro(vdd_min);
define_one_ro(vdd_max);
define_one_rw(vdd);
#endif
define_one_ro(vdd_freqs);

static struct attribute *default_attrs[] = {
#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
	&vdd.attr,
	&vdd_min.attr,
	&vdd_max.attr,
#endif
	&vdd_freqs.attr,
	NULL
};

static struct attribute_group override_attr_group = {
	.attrs = default_attrs,
	.name = "override"
};

static int __init cpufreq_override_driver_init(void)
{
	int ret = 0;

	nr_freqs = acpuclk_get_freqs(freq_table);
	printk("override: freqs configured: %u\n",nr_freqs);

	if((ret = sysfs_create_group(cpufreq_global_kobject, &override_attr_group)))
		 printk("override: failed!\n");
	else
		 printk("override: initialized!\n");

	return ret;
}

static void __exit cpufreq_override_driver_exit(void)
{
	sysfs_remove_group(cpufreq_global_kobject, &override_attr_group);
}

MODULE_AUTHOR("marco@unixpsycho.com");
MODULE_DESCRIPTION("'cpufreq_override' - A driver to do cool stuff ");
MODULE_LICENSE("GPL");

module_init(cpufreq_override_driver_init);
module_exit(cpufreq_override_driver_exit);


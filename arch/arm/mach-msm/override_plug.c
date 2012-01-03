/*
 *  override_plug.c
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

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
void acpuclk_get_voltages(unsigned int acpu_freqs_vlt_tbl[])
{
	int i=0;
	struct cpufreq_frequency_table *f;
	struct clkctl_acpu_speed *v;

	 v = acpu_freq_tbl;

	 for (f = freq_table; f->frequency != CPUFREQ_TABLE_END; f++) {
		while(v->acpu_clk_khz != f->frequency) v++;
		if(v->acpu_clk_khz == f->frequency)
			acpu_freqs_vlt_tbl[i] = v->vdd_mv;
		else printk("override: BUG!\n");
		i++;
	}
}
EXPORT_SYMBOL(acpuclk_get_voltages);

void acpuclk_set_voltages(unsigned int acpu_freqs_vlt_tbl[])
{
	int i=0;
	struct cpufreq_frequency_table *f;
	struct clkctl_acpu_speed *v;

	v = acpu_freq_tbl;

	for (f = freq_table; f->frequency != CPUFREQ_TABLE_END; f++) {
		while(v->acpu_clk_khz != f->frequency) v++;
		if(v->acpu_clk_khz == f->frequency) {
			v->vdd_mv = acpu_freqs_vlt_tbl[i];
			v->vdd_raw = VDD_RAW(v->vdd_mv);
		}
		else printk("override: BUG!\n");
		i++;
	}
}
EXPORT_SYMBOL(acpuclk_set_voltages);
#endif

unsigned int acpuclk_get_freqs(unsigned int acpu_freqs_tbl[])
{
	int i=0;
 	struct cpufreq_frequency_table *f;

	for (f = freq_table; f->frequency != CPUFREQ_TABLE_END; f++) {
		acpu_freqs_tbl[i] = f->frequency;
		i++;
	}

	return i;
}
EXPORT_SYMBOL(acpuclk_get_freqs);

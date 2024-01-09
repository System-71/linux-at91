// SPDX-License-Identifier: GPL-2.0-only
/*
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define MCK_RATE_200_MHZ 200000000
#define MCK_RATE_198_MHZ 198000000


struct sam9x60_cpu_dvfs_info {
	struct cpumask cpus;
	struct device *cpu_dev;
	struct clk *cpu_clk;
	struct clk *master_clk;
	struct list_head list_head;
};

struct clk {
        struct clk_core *core;
        struct device *dev;
        const char *dev_id;
        const char *con_id;
        unsigned long min_rate;
        unsigned long max_rate;
        unsigned int exclusive_count;
        struct hlist_node clks_node;
};

struct clk_core {
        const char              *name;
        const struct clk_ops    *ops;
        struct clk_hw           *hw;
        struct module           *owner;
        struct device           *dev;
        struct device_node      *of_node;
        struct clk_core         *parent;
        struct clk_parent_map   *parents;
        u8                      num_parents;
        u8                      new_parent_index;
        unsigned long           rate;
        unsigned long           req_rate;
        unsigned long           new_rate;
        struct clk_core         *new_parent;
        struct clk_core         *new_child;
        unsigned long           flags;
        bool                    orphan;
        bool                    rpm_enabled;
        unsigned int            enable_count;
        unsigned int            prepare_count;
        unsigned int            protect_count;
        unsigned long           min_rate;
        unsigned long           max_rate;
        unsigned long           accuracy;
        int                     phase;
        struct clk_duty         duty;
        struct hlist_head       children;
        struct hlist_node       child_node;
        struct hlist_head       clks;
        unsigned int            notifier_count;
#ifdef CONFIG_DEBUG_FS
        struct dentry           *dentry;
        struct hlist_node       debug_node;
#endif
        struct kref             ref;
};


static LIST_HEAD(dvfs_info_list);

static struct sam9x60_cpu_dvfs_info *sam9x60_cpu_dvfs_info_lookup(int cpu)
{
	struct sam9x60_cpu_dvfs_info *info;

	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (cpumask_test_cpu(cpu, &info->cpus))
			return info;
	}

	return NULL;
}

static int sam9x60_cpufreq_set_target(struct cpufreq_policy *policy,
				  unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	struct clk *cpu_clk = policy->clk;
	struct sam9x60_cpu_dvfs_info *info = policy->driver_data;
	struct device *cpu_dev = info->cpu_dev;
	struct dev_pm_opp *opp;
	long freq_hz, old_freq_hz;
	int ret;
	unsigned long master_rate;
	int count;
	int protected;

	old_freq_hz = clk_get_rate(cpu_clk);

	freq_hz = freq_table[index].frequency * 1000;

	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		pr_err("cpu%d: failed to find OPP for %ld\n",
		       policy->cpu, freq_hz);
		return PTR_ERR(opp);
	}

	/* Set the original PLL to target rate. */
	protected = (cpu_clk->core->protect_count > 0);
	if (protected) {
		pr_debug("cpu%d: CPU clock is protected\n", (int) policy->cpu);
		cpu_clk->core->protect_count--;
	}
	ret = clk_set_rate(cpu_clk, freq_hz);
	if (ret == EBUSY) {
		/* try again */
		ret = clk_set_rate(cpu_clk, freq_hz);
		if (ret) {
			pr_err("cpu%d: failed to scale cpu clock rate!\n",
		       	(int) policy->cpu);
			goto exit;
		}
	}

	master_rate = clk_get_rate(info->master_clk);
	pr_info("mck NEW rate=%ld\n", (long) master_rate);
exit:
	cpu_clk->core->protect_count = (protected == 1) ? 1 : 0;

	return ret;
}

static int sam9x60_cpufreq_data_init(struct sam9x60_cpu_dvfs_info* info, int cpu)
{	
	struct device *cpu_dev;
	struct clk *cpu_clk = ERR_PTR(-ENODEV);
	struct clk *master_clk = ERR_PTR(-ENODEV);
	int ret;

	cpu_dev = get_cpu_device(cpu);

	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, "plla");
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		pr_err("failed to get clk: plla\n");
		return ret;
	}
	master_clk = clk_get(cpu_dev, "mck");
	if (IS_ERR(master_clk)) {
		ret = PTR_ERR(master_clk);
		pr_err("failed to get clk: mck\n");
		goto out_free_resources;
	}

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, &info->cpus);
	if (ret) {
		pr_err("failed to get OPP-sharing information for cpu%d\n",
		       cpu);
		goto out_free_resources;
	}

	ret = dev_pm_opp_of_cpumask_add_table(&info->cpus);
	if (ret) {
		pr_warn("no OPP table for cpu%d\n", cpu);
		goto out_free_resources;
	}

	info->cpu_dev = cpu_dev;
	info->cpu_clk = cpu_clk;
	info->master_clk = master_clk;

	return 0;

out_free_resources:
	if (!IS_ERR(cpu_clk))
		clk_put(cpu_clk);
	if (!IS_ERR(master_clk))
		clk_put(master_clk);

	return ret;
}

static void sam9x60_data_release(struct sam9x60_cpu_dvfs_info *info)
{
	if (!IS_ERR(info->cpu_clk))
		clk_put(info->cpu_clk);
	if (!IS_ERR(info->master_clk))
		clk_put(info->master_clk);

	dev_pm_opp_of_cpumask_remove_table(&info->cpus);
}

static int sam9x60_cpufreq_init(struct cpufreq_policy *policy)
{
	struct sam9x60_cpu_dvfs_info *info;
	struct cpufreq_frequency_table *freq_table;
	int ret;

	info = sam9x60_cpu_dvfs_info_lookup(policy->cpu);
	if (!info) {
		pr_err("dvfs info for cpu%d is not initialized.\n",
		       policy->cpu);
		return -EINVAL;
	}

	ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table for cpu%d: %d\n",
		       policy->cpu, ret);
		return ret;
	}

	cpumask_copy(policy->cpus, &info->cpus);
	policy->freq_table = freq_table;
	policy->driver_data = info;
	policy->clk = info->cpu_clk;

	return 0;
}

static int sam9x60_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sam9x60_cpu_dvfs_info *info = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(info->cpu_dev, &policy->freq_table);

	return 0;
}

static struct cpufreq_driver sam9x60_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sam9x60_cpufreq_set_target,
	.get = cpufreq_generic_get,
	.init = sam9x60_cpufreq_init,
	.exit = sam9x60_cpufreq_exit,
	.register_em = cpufreq_register_em_with_opp,
	.name = "sam9x60-cpufreq",
	.attr = cpufreq_generic_attr,
};

static int sam9x60_cpufreq_probe(struct platform_device *pdev)
{
	struct sam9x60_cpu_dvfs_info *info, *tmp;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		info = sam9x60_cpu_dvfs_info_lookup(cpu);
		if (info)
			continue;

		info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			ret = -ENOMEM;
			goto release_dvfs_info_list;
		}

		ret = sam9x60_cpufreq_data_init(info, cpu);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to initialize dvfs info for cpu%d\n",
				cpu);
			goto release_dvfs_info_list;
		}

		list_add(&info->list_head, &dvfs_info_list);
	}

	ret = cpufreq_register_driver(&sam9x60_cpufreq_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed register driver: %d\n", ret);
		goto release_dvfs_info_list;
	}

	return 0;
release_dvfs_info_list:
	list_for_each_entry_safe(info, tmp, &dvfs_info_list, list_head) {
		sam9x60_data_release(info);
		list_del(&info->list_head);
	}
	return ret;
}

static struct platform_driver sam9x60_cpufreq_platdrv = {
	.driver = {
		.name	= "sam9x60-cpufreq",
	},
	.probe		= sam9x60_cpufreq_probe,
};

/* List of machines supported by this driver */
static const struct of_device_id at91_cpufreq_machines[] __initconst = {
	{ .compatible = "microchip,sam9x60", },

	{ }
};
MODULE_DEVICE_TABLE(of, at91_cpufreq_machines);

static int __init sam9x60_cpufreq_driver_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;
	struct platform_device *pdev;
	int err;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENODEV;

	match = of_match_node(at91_cpufreq_machines, np);
	of_node_put(np);
	if (!match) {
		pr_debug("Machine is not compatible with sam9x60-cpufreq\n");
		return -ENODEV;
	}

	err = platform_driver_register(&sam9x60_cpufreq_platdrv);
	if (err)
		return err;

	/*
	 * Since there's no place to hold device registration code and no
	 * device tree based way to match cpufreq driver yet, both the driver
	 * and the device registration codes are put here to handle defer
	 * probing.
	 */
	pdev = platform_device_register_simple("sam9x60-cpufreq", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("failed to register sam9x60-cpufreq platform device\n");
		platform_driver_unregister(&sam9x60_cpufreq_platdrv);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(sam9x60_cpufreq_driver_init);

MODULE_DESCRIPTION("AT91 Sam9x60 CPUFreq driver");
MODULE_AUTHOR("Mitchell Johnston <mitchelljohnston@hotmail.ca>");
MODULE_LICENSE("GPL v2");

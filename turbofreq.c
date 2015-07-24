#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/timer.h>
#include <linux/slab.h> /* kzalloc */
#include <linux/acpi.h> /* X86_* and ACPI_* macros */
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sched.h> /* task_struct current */
#include <linux/vmalloc.h>

#include <acpi/processor.h> /* ACPI_PROCESSOR_DEVICE_HID */

#include <asm/msr.h>
#include <asm/mwait.h>
#include <asm/msr-index.h>
#include <asm/cpufeature.h> /* cpu_has */
#include <asm/processor.h> /* boot_cpu_data */

#define	SAMPLE_RATE_MS	1000

#define FRAC_BITS 8
#define int_tofp(X) ((int64_t)(X) << FRAC_BITS)
#define fp_toint(X) ((X) >> FRAC_BITS)

static inline int32_t mul_fp(int32_t x, int32_t y)
{
	return ((int64_t)x * (int64_t)y) >> FRAC_BITS;
}

struct sample {
	int32_t core_pct_busy;
	u64 aperf;
	u64 mperf;
	u16 pstate;
	int freq;
	ktime_t time;
};

struct pstate_data {
	int	current_pstate;
	int	min_pstate;
	int	max_pstate;
	int	scaling;
	int	turbo_pstate;
};

struct cpudata {
	int cpu;

	struct timer_list timer;

	struct pstate_data pstate;

	ktime_t last_sample_time;
	u64	prev_aperf;
	u64	prev_mperf;
	struct sample sample;
	u16	stable_pstate; /* for per-cpu stable pstates */
	int	boost;
};

static struct cpudata **all_cpu_data;

/* for smp-wide pstate */
static u16 stable_pstate = 0;
static spinlock_t pstate_lock;
static u16 global_pstate = 0;

#define	POLICY_CPU	0
#define	POLICY_SMP	1
#define	POLICY_BOOST	2
#define	POLICY_NOTURBO	3
static int policy = POLICY_CPU;

static inline void intel_pstate_calc_busy(struct cpudata *cpu)
{
	u64 val;
	val = 100 * cpu->sample.aperf;
	do_div(val, cpu->sample.mperf);
	cpu->sample.core_pct_busy = val;
}

static int core_get_pstate(void)
{
	u64 perf_status;
	rdmsrl(MSR_IA32_PERF_STATUS, perf_status);
	return (perf_status & 0xffff) >> 8;
}

static int core_get_min_pstate(void)
{
	u64 value;

	rdmsrl(MSR_PLATFORM_INFO, value);
	return (value >> 40) & 0xFF;
}

static int core_get_max_pstate(void)
{
	u64 value;

	rdmsrl(MSR_PLATFORM_INFO, value);
	return (value >> 8) & 0xFF;
}

static int core_get_turbo_pstate(void)
{
	u64 value;
	int nont, ret;

	rdmsrl(MSR_NHM_TURBO_RATIO_LIMIT, value);
	nont = core_get_max_pstate();
	ret = (value) & 255;
	if (ret <= nont)
		ret = nont;
	return ret;
}

static inline int core_get_scaling(void)
{
	return 100000;
}

static void core_set_pstate(struct cpudata *cpudata, int pstate)
{
	u64 val;
	val = pstate << 8;
	wrmsrl_on_cpu(cpudata->cpu, MSR_IA32_PERF_CTL, val);
}

static inline void intel_pstate_sample(struct cpudata *cpu)
{
	u64 aperf, mperf;
	unsigned long flags;

	local_irq_save(flags);
	rdmsrl(MSR_IA32_APERF, aperf);
	rdmsrl(MSR_IA32_MPERF, mperf);
	local_irq_restore(flags);
	cpu->sample.pstate = core_get_pstate();

	cpu->last_sample_time = cpu->sample.time;
	cpu->sample.time = ktime_get();
	cpu->sample.aperf = aperf;
	cpu->sample.mperf = mperf;
	cpu->sample.aperf -= cpu->prev_aperf;
	cpu->sample.mperf -= cpu->prev_mperf;

	intel_pstate_calc_busy(cpu);

	cpu->prev_aperf = aperf;
	cpu->prev_mperf = mperf;
}

static void update_if_less(u16 pstate)
{
	spin_lock(&pstate_lock);
	if (stable_pstate > pstate || stable_pstate == 0)
		stable_pstate = pstate;
	spin_unlock(&pstate_lock);
}

static void intel_pstate_get_cpu_pstates(struct cpudata *cpu)
{
	cpu->pstate.min_pstate = core_get_min_pstate();
	cpu->pstate.max_pstate = core_get_max_pstate();
	cpu->pstate.turbo_pstate = core_get_turbo_pstate();
	cpu->pstate.scaling = core_get_scaling();

	core_set_pstate(cpu, cpu->pstate.turbo_pstate);
	cpu->stable_pstate = cpu->pstate.turbo_pstate;
	update_if_less(cpu->stable_pstate);
}

static void reset_pstates(void)
{
	unsigned long cpu;
	struct cpudata *c;

	spin_lock(&pstate_lock);
	stable_pstate = 0;
	global_pstate = 0;
	spin_unlock(&pstate_lock);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		c = all_cpu_data[cpu];
		intel_pstate_get_cpu_pstates(c);
		c->boost = 0;
	}
	put_online_cpus();

	return;
}

static void set_global_pstate(void)
{
	unsigned long cpu;
	struct cpudata *c;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		c = all_cpu_data[cpu];
		core_set_pstate(c, global_pstate);
	}
	put_online_cpus();

	return;
}

static void set_policy(struct cpudata *cpu, int policy)
{
	switch (policy) {
	case POLICY_CPU:
		core_set_pstate(cpu, cpu->stable_pstate);
		break;
	case POLICY_SMP:
		core_set_pstate(cpu, stable_pstate);
		break;
	case POLICY_BOOST:
		core_set_pstate(cpu, cpu->pstate.turbo_pstate);
		break;
	case POLICY_NOTURBO:
		core_set_pstate(cpu, cpu->pstate.max_pstate);
		break;
	}
}

static void adjust_turbo_pstate(struct cpudata *cpu)
{
	/* set minimum frequency within turbo boost range */
	if ((cpu->sample.pstate < cpu->stable_pstate) &&
	    (cpu->sample.pstate > cpu->pstate.max_pstate)) {
		cpu->stable_pstate = cpu->sample.pstate;
	}

	/* same thing for smp-wide pstate */
	if (cpu->sample.pstate > cpu->pstate.max_pstate) {
		update_if_less(cpu->sample.pstate);
	}

	/* now update core frequencies */

	/* first enforce global pstate */
	if (global_pstate) {
		set_global_pstate();
		return;
	}

	/* then enforce user space requested boosts */
	if (cpu->boost) {
		core_set_pstate(cpu, cpu->pstate.turbo_pstate);
		return;
	}

	/* finally enforce desired policy */
	set_policy(cpu, policy);
}

static void timer_func(unsigned long data)
{
	struct cpudata *cpu = (struct cpudata *)data;
	struct sample *sample;
	int delay;

	intel_pstate_sample(cpu);

	sample = &cpu->sample;

	/*if (cpu->cpu == 0)
		printk(KERN_ALERT "busy = %d max = %d turbo = %d current = %d min = %d\n",
		sample->core_pct_busy, cpu->pstate.max_pstate, cpu->pstate.turbo_pstate,
		sample->pstate, cpu->min_pstate);*/

	if (sample->core_pct_busy > 100) {
		adjust_turbo_pstate(cpu);
	} else {
		/* XXX default to performance, but ideally this case would mimick intel_pstate */
		core_set_pstate(cpu, cpu->pstate.turbo_pstate); 
	}

	delay = msecs_to_jiffies(SAMPLE_RATE_MS);
	mod_timer_pinned(&cpu->timer, jiffies + delay);
}

static int intel_pstate_init_cpu(unsigned int cpunum)
{
	struct cpudata *cpu;

	if (!all_cpu_data[cpunum])
		all_cpu_data[cpunum] = kzalloc(sizeof(struct cpudata),
					       GFP_KERNEL);
	if (!all_cpu_data[cpunum])
		return -ENOMEM;

	cpu = all_cpu_data[cpunum];

	cpu->cpu = cpunum;
	cpu->boost = 0;
	intel_pstate_get_cpu_pstates(cpu);

	init_timer_deferrable(&cpu->timer);
	cpu->timer.data = (unsigned long)cpu;
	cpu->timer.expires = jiffies + HZ/100;
	cpu->timer.function = timer_func;
	
	intel_pstate_sample(cpu);

	add_timer_on(&cpu->timer, cpunum);

	pr_debug("turbofreq cpufreq driver controlling: cpu %d\n", cpunum);

	return 0;
}


static int turbofreq_driver_verify(struct cpufreq_policy *policy)
{
	//cpufreq_verify_within_cpu_limits(policy);
	return 0;
}

static int turbofreq_driver_init(struct cpufreq_policy *policy)
{
	struct cpudata *cpu;
	int rc;

	rc = intel_pstate_init_cpu(policy->cpu);
	if (rc)
		return rc;

	cpu = all_cpu_data[policy->cpu];
	policy->min = cpu->pstate.min_pstate * cpu->pstate.scaling;
	policy->max = cpu->pstate.turbo_pstate * cpu->pstate.scaling;
	policy->cpuinfo.min_freq = cpu->pstate.min_pstate * cpu->pstate.scaling;
	policy->cpuinfo.max_freq = cpu->pstate.turbo_pstate * cpu->pstate.scaling;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	return 0;
}

static int turbofreq_driver_setpolicy(struct cpufreq_policy *policy)
{
	return 0;
}

static struct cpufreq_driver turbofreq_driver = {
	.name		= "turbofreq",
	.init		= turbofreq_driver_init,
	.verify		= turbofreq_driver_verify,
	.setpolicy	= turbofreq_driver_setpolicy,
};

/* sysfs begin */
static ssize_t pstate_available_policies_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "cpu smp boost noturbo\n");
}

static ssize_t pstate_policy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	switch (policy) {
		case POLICY_SMP:
			return scnprintf(buf, PAGE_SIZE, "smp\n");
		case POLICY_BOOST:
			return scnprintf(buf, PAGE_SIZE, "boost\n");
		case POLICY_NOTURBO:
			return scnprintf(buf, PAGE_SIZE, "noturbo\n");
		case POLICY_CPU:
		default:
			return scnprintf(buf, PAGE_SIZE, "cpu\n");
	}

}

static ssize_t pstate_policy_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cpudata *cpu;
	unsigned long int cpunum;

	if (!strncmp(buf, "cpu\n", count)) {
		policy = POLICY_CPU;
	} else if (!strncmp(buf, "smp\n", count)) {
		policy = POLICY_SMP;
	} else if (!strncmp(buf, "boost\n", count)) {
		policy = POLICY_BOOST;
	} else if (!strncmp(buf, "noturbo\n", count)) {
		policy = POLICY_NOTURBO;
	} else {
		return -EINVAL;
	}

	/* XXX set per-cpu pstate now or just wait for next timer_func call? */
	cpunum = dev->id;
	cpu = all_cpu_data[cpunum];
	set_policy(cpu, policy);

	return count;
}

static ssize_t task_boost_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct cpudata *cpu;
	int cpunum;

	cpunum = task_cpu(current); /* XXX assumes task is pinned, no cpu migration */
	cpu = all_cpu_data[cpunum];
	return scnprintf(buf, PAGE_SIZE, "%d\n", cpu->boost);
}

static ssize_t task_boost_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	unsigned int boost;
	int ret;
	unsigned long cpunum;
	struct cpudata *cpu;

	ret = sscanf(buf, "%u", &boost);
	if (ret != 1)
		return -EINVAL;

	if (boost > 1) /* boost is binary for now */
		return -EINVAL;

	cpunum = task_cpu(current); /* XXX assumes task is pinned, no cpu migration */
	cpu = all_cpu_data[cpunum];
	cpu->boost = boost;

	return count;
}

static ssize_t reset_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n");	
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	reset_pstates();
	return count;
}

static ssize_t global_pstate_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", global_pstate);
}

static ssize_t global_pstate_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1)
		return -EINVAL;

	if (val > 255) /* frequency field is 8-bits anway */
		return -EINVAL;

	/* XXX check for valid frequency */
	if (global_pstate == 0 || val < global_pstate) {
		global_pstate = val;
		set_global_pstate(val);
	}
	return count;
}


static DEVICE_ATTR_RW(pstate_policy);
static DEVICE_ATTR_RW(task_boost);
static DEVICE_ATTR_RW(reset);
static DEVICE_ATTR_RW(global_pstate);
static DEVICE_ATTR_RO(pstate_available_policies);

static struct attribute *turbofreq_attrs[] = {
	&dev_attr_pstate_policy.attr,
	&dev_attr_reset.attr,
	&dev_attr_task_boost.attr,
	&dev_attr_global_pstate.attr,
	&dev_attr_pstate_available_policies.attr,
	NULL
};

static struct attribute_group turbofreq_attr_group = {
	.attrs = turbofreq_attrs,
	.name = "turbofreq"
};

static int turbofreq_add_interface(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &turbofreq_attr_group);
}

/* per-cpu interface */
static ssize_t boost_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	unsigned int boost;
	int ret;
	struct cpudata *cpu;
	int cpunum;

	ret = sscanf(buf, "%u", &boost);
	if (ret != 1)
		return -EINVAL;

	if (boost > 1) /* boost is binary for now */
		return -EINVAL;

	cpunum = dev->id; /* XXX assumes task is pinned, no cpu migration */
	cpu = all_cpu_data[cpunum];
	cpu->boost = boost;

	return count;
}

static ssize_t boost_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct cpudata *cpu;
	int cpunum;

	cpunum = dev->id; /* XXX assumes task is pinned, no cpu migration */
	cpu = all_cpu_data[cpunum];
	return scnprintf(buf, PAGE_SIZE, "%d\n", cpu->boost);
}

static DEVICE_ATTR_RW(boost);

static int turbofreq_cpu_add_interface(unsigned long cpu)
{
	struct device *dev = get_cpu_device(cpu);
	return sysfs_create_file(&dev->kobj, &dev_attr_boost.attr);
}
/* sysfs end */

static void __turbofreq_exit(void)
{
	vfree(all_cpu_data);
}

static int __init turbofreq_init(void)
{
	int rc;
	unsigned long cpu;

	//XXX missing driver cpu matching
	//XXX missing cpu hotplug mess
	all_cpu_data = vzalloc(sizeof(void*) * num_possible_cpus());
	if (!all_cpu_data)
		return -ENOMEM;

	spin_lock_init(&pstate_lock);

	rc = turbofreq_add_interface(cpu_subsys.dev_root);
	if (rc)
		goto out;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		turbofreq_cpu_add_interface(cpu);
	}
	put_online_cpus();

	rc = cpufreq_register_driver(&turbofreq_driver);
	if (rc) 
		goto out;

	return rc;
out: 
	__turbofreq_exit();
	return rc;
}
module_init(turbofreq_init);

static void __exit turbofreq_exit(void)
{
	/* actually we can't remove this driver, but whatever */
	__turbofreq_exit();
}
module_exit(turbofreq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pablo Llopis <pablo.llopis@gmail.com>");
MODULE_DESCRIPTION("Adapt Turbo-Boost Frequency to minimize variation for Intel CPUs");

static const struct x86_cpu_id acpi_cpufreq_ids[] = {
	X86_FEATURE_MATCH(X86_FEATURE_ACPI),
	X86_FEATURE_MATCH(X86_FEATURE_HW_PSTATE),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, acpi_cpufreq_ids);

static const struct acpi_device_id processor_device_ids[] = {
	{ACPI_PROCESSOR_OBJECT_HID, },
	{ACPI_PROCESSOR_DEVICE_HID, },
	{},
};
MODULE_DEVICE_TABLE(acpi, processor_device_ids);


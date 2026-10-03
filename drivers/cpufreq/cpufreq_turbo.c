// SPDX-License-Identifier: GPL-2.0
/*
 * cpufreq_turbo.c - governor "turbo"
 *
 * Copyright (c) 2026 Hinomori Project
 *
 * Mengunci frekuensi CPU ke target tinggi dan MEMAKSA balik kalau ada pihak
 * lain (thermal vendor, boost driver, firmware) menurunkannya. Beda dengan
 * "performance" yang hanya set frekuensi sekali, turbo:
 *
 *   - re-assert periodik (reassert_ms)
 *   - target bisa persen dari max (target_pct) atau frekuensi tetap (target_khz)
 *   - thermal guard opsional: kalau suhu zone > batas, target diturunkan
 *     bertahap, lalu dinaikkan lagi saat sudah dingin
 *   - semua tunable per-policy (per-cluster) di
 *     /sys/devices/system/cpu/cpufreq/policyN/turbo/
 *
 * Target kernel: 5.10 (android12/13-5.10 GKI).
 */

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#define TURBO_DEF_PCT		100
#define TURBO_DEF_REASSERT_MS	100
#define TURBO_DEF_STEP		10
#define TURBO_MAX_THROTTLE	90
#define TURBO_TZ_LEN		32

struct turbo_policy {
	struct kobject kobj;
	struct cpufreq_policy *policy;
	struct delayed_work work;
	struct mutex lock;
	bool active;

	/* tunables */
	unsigned int target_pct;	/* 1-100, persen dari policy->max */
	unsigned int target_khz;	/* 0 = pakai target_pct */
	unsigned int reassert_ms;	/* 0 = tanpa re-assert (seperti performance) */
	unsigned int thermal_limit;	/* derajat C, 0 = thermal guard mati */
	unsigned int thermal_step;	/* persen penurunan per periode */
	char thermal_zone[TURBO_TZ_LEN];/* nama (type) thermal zone */

	/* state / debug */
	unsigned int throttle;		/* persen yang sedang dipotong */
	unsigned int last_target;	/* kHz terakhir yang diminta */
};

/* ------------------------------------------------------------------ */
/* inti: hitung target dan terapkan                                    */
/* ------------------------------------------------------------------ */

static void turbo_update_throttle(struct turbo_policy *tp)
{
	struct thermal_zone_device *tz;
	int temp, limit_mc;

	if (!tp->thermal_limit || !tp->thermal_zone[0]) {
		tp->throttle = 0;
		return;
	}

	tz = thermal_zone_get_zone_by_name(tp->thermal_zone);
	if (IS_ERR(tz) || thermal_zone_get_temp(tz, &temp))
		return;

	limit_mc = tp->thermal_limit * 1000;
	if (temp > limit_mc) {
		tp->throttle = min_t(unsigned int, tp->throttle + tp->thermal_step,
				     TURBO_MAX_THROTTLE);
	} else if (temp < limit_mc - 3000) {
		tp->throttle = tp->throttle > tp->thermal_step ?
			       tp->throttle - tp->thermal_step : 0;
	}
}

static unsigned int turbo_target(struct turbo_policy *tp)
{
	struct cpufreq_policy *p = tp->policy;
	unsigned int f;

	if (tp->target_khz)
		f = tp->target_khz;
	else
		f = div_u64((u64)p->max * tp->target_pct, 100);

	if (tp->throttle)
		f = div_u64((u64)f * (100 - tp->throttle), 100);

	return clamp(f, p->min, p->max);
}

/* tp->lock harus dipegang */
static void turbo_apply_locked(struct turbo_policy *tp)
{
	unsigned int target;

	turbo_update_throttle(tp);
	target = turbo_target(tp);
	tp->last_target = target;
	__cpufreq_driver_target(tp->policy, target, CPUFREQ_RELATION_H);
}

static void turbo_work(struct work_struct *w)
{
	struct turbo_policy *tp = container_of(to_delayed_work(w),
					       struct turbo_policy, work);
	unsigned int ms = 0;

	mutex_lock(&tp->lock);
	if (tp->active) {
		turbo_apply_locked(tp);
		ms = tp->reassert_ms;
	}
	mutex_unlock(&tp->lock);

	if (ms)
		schedule_delayed_work(&tp->work, msecs_to_jiffies(max(ms, 10u)));
}

/* jalankan work segera (dipakai setelah tunable berubah) */
static void turbo_kick(struct turbo_policy *tp)
{
	mutex_lock(&tp->lock);
	if (tp->active)
		mod_delayed_work(system_wq, &tp->work, 0);
	mutex_unlock(&tp->lock);
}

/* ------------------------------------------------------------------ */
/* sysfs                                                               */
/* ------------------------------------------------------------------ */

struct turbo_attr {
	struct attribute attr;
	ssize_t (*show)(struct turbo_policy *tp, char *buf);
	ssize_t (*store)(struct turbo_policy *tp, const char *buf, size_t count);
};

#define to_tp(k)	container_of(k, struct turbo_policy, kobj)
#define to_tattr(a)	container_of(a, struct turbo_attr, attr)

static ssize_t turbo_attr_show(struct kobject *k, struct attribute *a, char *buf)
{
	struct turbo_attr *ta = to_tattr(a);

	return ta->show ? ta->show(to_tp(k), buf) : -EIO;
}

static ssize_t turbo_attr_store(struct kobject *k, struct attribute *a,
				const char *buf, size_t count)
{
	struct turbo_attr *ta = to_tattr(a);

	return ta->store ? ta->store(to_tp(k), buf, count) : -EIO;
}

static const struct sysfs_ops turbo_sysfs_ops = {
	.show	= turbo_attr_show,
	.store	= turbo_attr_store,
};

#define TURBO_SHOW(name)						\
static ssize_t name##_show(struct turbo_policy *tp, char *buf)		\
{									\
	return scnprintf(buf, PAGE_SIZE, "%u\n", tp->name);		\
}

#define TURBO_STORE(name, lo, hi)					\
static ssize_t name##_store(struct turbo_policy *tp, const char *buf,	\
			    size_t count)				\
{									\
	unsigned int v;							\
	int ret = kstrtouint(buf, 10, &v);				\
	if (ret)							\
		return ret;						\
	if (v < (lo) || v > (hi))					\
		return -EINVAL;						\
	mutex_lock(&tp->lock);						\
	tp->name = v;							\
	mutex_unlock(&tp->lock);					\
	turbo_kick(tp);							\
	return count;							\
}

#define TURBO_RW(name, lo, hi)						\
	TURBO_SHOW(name) TURBO_STORE(name, lo, hi)			\
	static struct turbo_attr turbo_attr_##name =			\
		__ATTR(name, 0644, name##_show, name##_store)

#define TURBO_RO(name)							\
	TURBO_SHOW(name)						\
	static struct turbo_attr turbo_attr_##name =			\
		__ATTR(name, 0444, name##_show, NULL)

TURBO_RW(target_pct, 1, 100);
TURBO_RW(target_khz, 0, 10000000);
TURBO_RW(reassert_ms, 0, 60000);
TURBO_RW(thermal_limit, 0, 120);
TURBO_RW(thermal_step, 1, 50);
TURBO_RO(throttle);
TURBO_RO(last_target);

static ssize_t thermal_zone_show(struct turbo_policy *tp, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", tp->thermal_zone);
}

static ssize_t thermal_zone_store(struct turbo_policy *tp, const char *buf,
				  size_t count)
{
	char tmp[TURBO_TZ_LEN] = "";

	/* kosong / baris baru saja = matikan */
	if (count > 0 && buf[0] != '\n' && sscanf(buf, "%31s", tmp) != 1)
		return -EINVAL;

	mutex_lock(&tp->lock);
	strscpy(tp->thermal_zone, tmp, sizeof(tp->thermal_zone));
	tp->throttle = 0;
	mutex_unlock(&tp->lock);
	turbo_kick(tp);
	return count;
}

static struct turbo_attr turbo_attr_thermal_zone =
	__ATTR(thermal_zone, 0644, thermal_zone_show, thermal_zone_store);

static struct attribute *turbo_attrs[] = {
	&turbo_attr_target_pct.attr,
	&turbo_attr_target_khz.attr,
	&turbo_attr_reassert_ms.attr,
	&turbo_attr_thermal_zone.attr,
	&turbo_attr_thermal_limit.attr,
	&turbo_attr_thermal_step.attr,
	&turbo_attr_throttle.attr,
	&turbo_attr_last_target.attr,
	NULL
};
ATTRIBUTE_GROUPS(turbo);

static void turbo_release(struct kobject *k)
{
	kfree(to_tp(k));
}

static struct kobj_type turbo_ktype = {
	.release	= turbo_release,
	.sysfs_ops	= &turbo_sysfs_ops,
	.default_groups	= turbo_groups,
};

/* ------------------------------------------------------------------ */
/* callback governor                                                   */
/* ------------------------------------------------------------------ */

static int turbo_init(struct cpufreq_policy *policy)
{
	struct turbo_policy *tp;
	int ret;

	tp = kzalloc(sizeof(*tp), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;

	tp->policy = policy;
	tp->target_pct = TURBO_DEF_PCT;
	tp->reassert_ms = TURBO_DEF_REASSERT_MS;
	tp->thermal_step = TURBO_DEF_STEP;
	mutex_init(&tp->lock);
	INIT_DELAYED_WORK(&tp->work, turbo_work);

	ret = kobject_init_and_add(&tp->kobj, &turbo_ktype, &policy->kobj, "turbo");
	if (ret) {
		kobject_put(&tp->kobj);	/* release() akan kfree */
		return ret;
	}

	policy->governor_data = tp;
	return 0;
}

static void turbo_exit(struct cpufreq_policy *policy)
{
	struct turbo_policy *tp = policy->governor_data;

	policy->governor_data = NULL;
	kobject_put(&tp->kobj);
}

static int turbo_start(struct cpufreq_policy *policy)
{
	struct turbo_policy *tp = policy->governor_data;
	unsigned int ms;

	mutex_lock(&tp->lock);
	tp->active = true;
	tp->throttle = 0;
	ms = tp->reassert_ms;
	mutex_unlock(&tp->lock);

	/* limits() dipanggil core setelah start() dan menerapkan frekuensi pertama */
	if (ms)
		schedule_delayed_work(&tp->work, msecs_to_jiffies(max(ms, 10u)));
	return 0;
}

static void turbo_stop(struct cpufreq_policy *policy)
{
	struct turbo_policy *tp = policy->governor_data;

	mutex_lock(&tp->lock);
	tp->active = false;
	mutex_unlock(&tp->lock);

	cancel_delayed_work_sync(&tp->work);
}

static void turbo_limits(struct cpufreq_policy *policy)
{
	struct turbo_policy *tp = policy->governor_data;

	mutex_lock(&tp->lock);
	if (tp->active)
		turbo_apply_locked(tp);
	mutex_unlock(&tp->lock);
}

static struct cpufreq_governor turbo_gov = {
	.name	= "turbo",
	.owner	= THIS_MODULE,
	.flags	= CPUFREQ_GOV_STRICT_TARGET,
	.init	= turbo_init,
	.exit	= turbo_exit,
	.start	= turbo_start,
	.stop	= turbo_stop,
	.limits	= turbo_limits,
};

static int __init cpufreq_turbo_init(void)
{
	return cpufreq_register_governor(&turbo_gov);
}

static void __exit cpufreq_turbo_exit(void)
{
	cpufreq_unregister_governor(&turbo_gov);
}

MODULE_AUTHOR("Hinomori Project");
MODULE_DESCRIPTION("CPUfreq governor 'turbo': kunci frekuensi tinggi dengan re-assert dan thermal guard");
MODULE_LICENSE("GPL");

core_initcall(cpufreq_turbo_init);
module_exit(cpufreq_turbo_exit);

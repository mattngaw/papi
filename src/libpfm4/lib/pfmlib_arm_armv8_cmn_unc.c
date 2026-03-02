/*
 * pfmlib_arm_armv8_cmn_unc.c : support for Arm CMN-600 uncore PMUs
 *
 * Copyright (c) 2026 Google Inc. All rights reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* private headers */
#include "pfmlib_priv.h"
#include "pfmlib_arm_priv.h"
#include "pfmlib_arm_armv8_unc_priv.h"

#ifndef PERF_ATTR_U
#define PERF_ATTR_U 0
#define PERF_ATTR_K 1
#define PERF_ATTR_H 2
#define PERF_ATTR_PE 3
#define PERF_ATTR_FR 4
#define PERF_ATTR_PR 5
#define PERF_ATTR_MG 7
#define PERF_ATTR_MH 8
#define PERF_ATTR_HWS 11
#endif

#define ARM_CMN_SYSFS_PMUS_DIR "/sys/bus/event_source/devices"
#define ARM_CMN_PMU_PREFIX "arm_cmn_"
#define ARM_CMN_MAX_INSTANCES 2
#define ARM_CMN_MAX_LINE 1024
#define ARM_CMN_CODE_COUNT 3
#define ARM_CMN_WATCHPOINT_TYPE 0x7770ULL

enum arm_cmn_attr_idx {
	ARM_CMN_ATTR_TYPE = 0,
	ARM_CMN_ATTR_EVENTID,
	ARM_CMN_ATTR_OCCUPID,
	ARM_CMN_ATTR_WP_COMBINE,
	ARM_CMN_ATTR_BYNODEID,
	ARM_CMN_ATTR_NODEID,
	ARM_CMN_ATTR_WP_DEV_SEL,
	ARM_CMN_ATTR_WP_CHN_SEL,
	ARM_CMN_ATTR_WP_GRP,
	ARM_CMN_ATTR_WP_EXCLUSIVE,
	ARM_CMN_ATTR_WP_MASK,
	ARM_CMN_ATTR_WP_VAL,
	ARM_CMN_ATTR_MAX
};

#define ARM_CMN_ATTR_BIT(x) (1ULL << (x))

#define ARM_CMN_WATCHPOINT_ATTRS \
	(ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_COMBINE) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_DEV_SEL) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_CHN_SEL) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_GRP) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_EXCLUSIVE) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_MASK) | \
	 ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_WP_VAL))

static const pfmlib_attr_desc_t arm_cmn_mods[] = {
	PFM_ATTR_I("type", "type [16 bits]"),
	PFM_ATTR_I("eventid", "eventid [11 bits]"),
	PFM_ATTR_I("occupid", "occupid [4 bits]"),
	PFM_ATTR_I("wp_combine", "wp_combine [4 bits]"),
	PFM_ATTR_B("bynodeid", "bynodeid"),
	PFM_ATTR_I("nodeid", "nodeid [16 bits]"),
	PFM_ATTR_I("wp_dev_sel", "wp_dev_sel [3 bits]"),
	PFM_ATTR_I("wp_chn_sel", "wp_chn_sel [5 bits]"),
	PFM_ATTR_B("wp_grp", "wp_grp"),
	PFM_ATTR_B("wp_exclusive", "wp_exclusive"),
	PFM_ATTR_I("wp_mask", "wp_mask [64 bits]"),
	PFM_ATTR_I("wp_val", "wp_val [64 bits]"),
	PFM_ATTR_NULL
};

typedef struct {
	int valid;
	unsigned int reg;
	unsigned int lsb;
	unsigned int width;
} arm_cmn_field_t;

typedef struct {
	const char *name;
	const char *desc;
	uint64_t dfl_codes[ARM_CMN_CODE_COUNT];
	uint64_t modmsk;
} arm_cmn_entry_t;

typedef struct {
	int initialized;
	int count;
	arm_cmn_entry_t *events;
	arm_cmn_field_t fields[ARM_CMN_ATTR_MAX];
} arm_cmn_event_table_t;

static arm_cmn_event_table_t arm_cmn_event_tables[ARM_CMN_MAX_INSTANCES];

static int
arm_cmn_attr_from_name(const char *name)
{
	int i;

	if (!name)
		return -1;

	for (i = 0; i < ARM_CMN_ATTR_MAX; i++) {
		if (!strcmp(name, arm_cmn_mods[i].name))
			return i;
	}

	return -1;
}

static char *
arm_cmn_trim(char *s)
{
	char *end;

	if (!s)
		return s;

	while (*s && isspace((unsigned char)*s))
		s++;

	end = s + strlen(s);
	while (end > s && isspace((unsigned char)end[-1]))
		*--end = '\0';

	return s;
}

static int
arm_cmn_filter_regular_file(const struct dirent *d)
{
	if (!d || d->d_name[0] == '.')
		return 0;

	if (d->d_type == DT_DIR)
		return 0;

	return 1;
}

static int
arm_cmn_get_instance_id(const char *pmu_name)
{
	const size_t prefix_len = sizeof(ARM_CMN_PMU_PREFIX) - 1;
	char *endptr;
	long instance;

	if (!pmu_name)
		return -1;

	if (strncmp(pmu_name, ARM_CMN_PMU_PREFIX, prefix_len))
		return -1;

	instance = strtol(pmu_name + prefix_len, &endptr, 10);
	if (!endptr || *endptr != '\0')
		return -1;

	if (instance < 0 || instance >= ARM_CMN_MAX_INSTANCES)
		return -1;

	return (int)instance;
}

static int
arm_cmn_read_line(const char *path, char *buf, size_t len)
{
	FILE *fp;
	size_t n;

	if (!path || !buf || !len)
		return PFM_ERR_INVAL;

	fp = fopen(path, "r");
	if (!fp)
		return PFM_ERR_NOTSUPP;

	if (!fgets(buf, len, fp)) {
		fclose(fp);
		return PFM_ERR_NOTSUPP;
	}

	fclose(fp);

	n = strlen(buf);
	while (n && isspace((unsigned char)buf[n - 1]))
		buf[--n] = '\0';

	return n ? PFM_SUCCESS : PFM_ERR_NOTSUPP;
}

static int
arm_cmn_parse_u64(const char *s, uint64_t *value)
{
	char *endptr = NULL;
	unsigned long long tmp;

	if (!s || !value)
		return PFM_ERR_INVAL;

	errno = 0;
	tmp = strtoull(s, &endptr, 0);
	if (errno || endptr == s || *endptr != '\0')
		return PFM_ERR_ATTR_VAL;

	*value = (uint64_t)tmp;
	return PFM_SUCCESS;
}

static uint64_t
arm_cmn_field_mask(const arm_cmn_field_t *field)
{
	if (field->width == 64)
		return ~0ULL;

	return (1ULL << field->width) - 1ULL;
}

static int
arm_cmn_set_field_value(uint64_t *codes, const arm_cmn_field_t *field, uint64_t value)
{
	uint64_t mask;
	uint64_t shifted_mask;

	if (!codes || !field || !field->valid)
		return PFM_ERR_ATTR;

	if (field->reg >= ARM_CMN_CODE_COUNT)
		return PFM_ERR_ATTR;

	mask = arm_cmn_field_mask(field);
	if (field->width < 64 && value > mask)
		return PFM_ERR_ATTR_VAL;

	if (field->width == 64) {
		if (field->lsb != 0)
			return PFM_ERR_ATTR;

		codes[field->reg] = value;
		return PFM_SUCCESS;
	}

	shifted_mask = mask << field->lsb;
	codes[field->reg] &= ~shifted_mask;
	codes[field->reg] |= (value & mask) << field->lsb;

	return PFM_SUCCESS;
}

static int
arm_cmn_get_field_value(const uint64_t *codes, const arm_cmn_field_t *field, uint64_t *value)
{
	uint64_t mask;

	if (!codes || !field || !field->valid || !value)
		return PFM_ERR_ATTR;

	if (field->reg >= ARM_CMN_CODE_COUNT)
		return PFM_ERR_ATTR;

	if (field->width == 64) {
		if (field->lsb != 0)
			return PFM_ERR_ATTR;

		*value = codes[field->reg];
		return PFM_SUCCESS;
	}

	mask = arm_cmn_field_mask(field);
	*value = (codes[field->reg] >> field->lsb) & mask;
	return PFM_SUCCESS;
}

static int
arm_cmn_parse_format(const char *spec, arm_cmn_field_t *field)
{
	char reg_name[16];
	unsigned int lo, hi;
	int n;

	if (!spec || !field)
		return PFM_ERR_INVAL;

	if (strchr(spec, ','))
		return PFM_ERR_NOTSUPP;

	n = sscanf(spec, "%15[^:]:%u-%u", reg_name, &lo, &hi);
	if (n == 2)
		hi = lo;
	else if (n != 3)
		return PFM_ERR_INVAL;

	if (!strcmp(reg_name, "config"))
		field->reg = 0;
	else if (!strcmp(reg_name, "config1"))
		field->reg = 1;
	else if (!strcmp(reg_name, "config2"))
		field->reg = 2;
	else
		return PFM_ERR_NOTSUPP;

	if (hi < lo || hi > 63)
		return PFM_ERR_INVAL;

	field->lsb = lo;
	field->width = hi - lo + 1;

	if (!field->width || field->width > 64)
		return PFM_ERR_INVAL;

	if (field->width == 64 && field->lsb != 0)
		return PFM_ERR_INVAL;

	field->valid = 1;

	return PFM_SUCCESS;
}

static int
arm_cmn_load_format_table(const char *pmu_name, arm_cmn_event_table_t *table)
{
	struct dirent **format_files = NULL;
	char format_dir[PATH_MAX];
	char format_path[PATH_MAX];
	char format_spec[ARM_CMN_MAX_LINE];
	int nfiles;
	int i;
	int ret = PFM_SUCCESS;

	if (!pmu_name || !table)
		return PFM_ERR_INVAL;

	if (snprintf(format_dir, sizeof(format_dir), "%s/%s/format",
		     ARM_CMN_SYSFS_PMUS_DIR, pmu_name) >= (int)sizeof(format_dir)) {
		return PFM_ERR_NOTSUPP;
	}

	memset(table->fields, 0, sizeof(table->fields));

	nfiles = scandir(format_dir, &format_files, arm_cmn_filter_regular_file, alphasort);
	if (nfiles <= 0)
		return PFM_ERR_NOTSUPP;

	for (i = 0; i < nfiles; i++) {
		int attr_idx;

		attr_idx = arm_cmn_attr_from_name(format_files[i]->d_name);
		if (attr_idx < 0)
			continue;

		if (snprintf(format_path, sizeof(format_path), "%s/%s", format_dir,
		     format_files[i]->d_name) >= (int)sizeof(format_path)) {
			ret = PFM_ERR_NOTSUPP;
			goto out;
		}

		ret = arm_cmn_read_line(format_path, format_spec, sizeof(format_spec));
		if (ret != PFM_SUCCESS)
			goto out;

		ret = arm_cmn_parse_format(format_spec, &table->fields[attr_idx]);
		if (ret != PFM_SUCCESS)
			goto out;
	}

	if (!table->fields[ARM_CMN_ATTR_TYPE].valid
	    || !table->fields[ARM_CMN_ATTR_EVENTID].valid
	    || !table->fields[ARM_CMN_ATTR_BYNODEID].valid
	    || !table->fields[ARM_CMN_ATTR_NODEID].valid)
		ret = PFM_ERR_NOTSUPP;

out:
	if (format_files) {
		for (i = 0; i < nfiles; i++)
			free(format_files[i]);
		free(format_files);
	}

	return ret;
}

static int
arm_cmn_is_watchpoint_event(arm_cmn_event_table_t *table, arm_cmn_entry_t *entry)
{
	uint64_t type;

	if (!table || !entry)
		return 0;

	if (arm_cmn_get_field_value(entry->dfl_codes,
		    &table->fields[ARM_CMN_ATTR_TYPE], &type) != PFM_SUCCESS)
		return 0;

	return type == ARM_CMN_WATCHPOINT_TYPE;
}

static int
arm_cmn_build_event_entry(arm_cmn_event_table_t *table, arm_cmn_entry_t *entry,
			  const char *spec)
{
	char *tmp = NULL;
	char *saveptr = NULL;
	char *tok;
	int is_watchpoint;
	int ret = PFM_SUCCESS;

	if (!table || !entry || !spec)
		return PFM_ERR_INVAL;

	memset(entry->dfl_codes, 0, sizeof(entry->dfl_codes));

	entry->modmsk = 0;
	if (table->fields[ARM_CMN_ATTR_BYNODEID].valid)
		entry->modmsk |= ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_BYNODEID);
	if (table->fields[ARM_CMN_ATTR_NODEID].valid)
		entry->modmsk |= ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_NODEID);

	tmp = strdup(spec);
	if (!tmp)
		return PFM_ERR_NOMEM;

	for (tok = strtok_r(tmp, ",", &saveptr);
	     tok;
	     tok = strtok_r(NULL, ",", &saveptr)) {
		char *eq;
		char *key;
		char *val;
		int attr_idx;
		uint64_t parsed;

		tok = arm_cmn_trim(tok);
		if (!*tok)
			continue;

		eq = strchr(tok, '=');
		if (!eq) {
			ret = PFM_ERR_INVAL;
			goto out;
		}

		*eq = '\0';
		key = arm_cmn_trim(tok);
		val = arm_cmn_trim(eq + 1);

		attr_idx = arm_cmn_attr_from_name(key);
		if (attr_idx < 0) {
			ret = PFM_ERR_NOTSUPP;
			goto out;
		}

		if (!table->fields[attr_idx].valid) {
			ret = PFM_ERR_NOTSUPP;
			goto out;
		}

		entry->modmsk |= ARM_CMN_ATTR_BIT(attr_idx);

		if (!strcmp(val, "?"))
			continue;

		ret = arm_cmn_parse_u64(val, &parsed);
		if (ret != PFM_SUCCESS)
			goto out;

		ret = arm_cmn_set_field_value(entry->dfl_codes,
					      &table->fields[attr_idx], parsed);
		if (ret != PFM_SUCCESS)
			goto out;
	}

	is_watchpoint = arm_cmn_is_watchpoint_event(table, entry);
	if (is_watchpoint)
		entry->modmsk &= ~ARM_CMN_ATTR_BIT(ARM_CMN_ATTR_OCCUPID);

	if (is_watchpoint)
		entry->modmsk |= ARM_CMN_WATCHPOINT_ATTRS;

	entry->modmsk &= ~(~0ULL << ARM_CMN_ATTR_MAX);

out:
	free(tmp);
	return ret;
}

static void
arm_cmn_free_table(arm_cmn_event_table_t *table)
{
	int i;

	if (!table)
		return;

	if (table->events) {
		for (i = 0; i < table->count; i++) {
			free((char *)table->events[i].name);
			free((char *)table->events[i].desc);
		}
		free(table->events);
	}

	table->events = NULL;
	table->count = 0;
	table->initialized = 0;
	memset(table->fields, 0, sizeof(table->fields));
}

static int
arm_cmn_load_table(pfmlib_pmu_t *pmu, int instance)
{
	arm_cmn_event_table_t *table;
	struct dirent **event_files = NULL;
	char events_dir[PATH_MAX];
	char event_path[PATH_MAX];
	char spec[ARM_CMN_MAX_LINE];
	int i, nfiles, nevents;
	int ret = PFM_SUCCESS;

	if (!pmu || instance < 0 || instance >= ARM_CMN_MAX_INSTANCES)
		return PFM_ERR_NOTSUPP;

	table = &arm_cmn_event_tables[instance];
	if (table->initialized) {
		if (!table->count || !table->events)
			return PFM_ERR_NOTSUPP;

		pmu->pe = table->events;
		pmu->pme_count = table->count;
		return PFM_SUCCESS;
	}

	ret = arm_cmn_load_format_table(pmu->perf_name, table);
	if (ret != PFM_SUCCESS)
		return ret;

	if (snprintf(events_dir, sizeof(events_dir), "%s/%s/events",
		     ARM_CMN_SYSFS_PMUS_DIR, pmu->perf_name) >= (int)sizeof(events_dir)) {
		return PFM_ERR_NOTSUPP;
	}

	nfiles = scandir(events_dir, &event_files, arm_cmn_filter_regular_file, alphasort);
	if (nfiles <= 0)
		return PFM_ERR_NOTSUPP;

	table->events = calloc((size_t)nfiles, sizeof(*table->events));
	if (!table->events) {
		ret = PFM_ERR_NOMEM;
		goto out;
	}

	nevents = 0;
	for (i = 0; i < nfiles; i++) {
		arm_cmn_entry_t *evt;

		if (snprintf(event_path, sizeof(event_path), "%s/%s", events_dir,
		     event_files[i]->d_name) >= (int)sizeof(event_path))
			continue;

		if (arm_cmn_read_line(event_path, spec, sizeof(spec)) != PFM_SUCCESS)
			continue;

		evt = &table->events[nevents];
		ret = arm_cmn_build_event_entry(table, evt, spec);
		if (ret != PFM_SUCCESS)
			continue;

		evt->name = strdup(event_files[i]->d_name);
		if (!evt->name) {
			ret = PFM_ERR_NOMEM;
			goto out;
		}

		evt->desc = strdup(event_files[i]->d_name);
		if (!evt->desc) {
			free((char *)evt->name);
			evt->name = NULL;
			ret = PFM_ERR_NOMEM;
			goto out;
		}

		nevents++;
	}

	table->count = nevents;
	if (!table->count) {
		ret = PFM_ERR_NOTSUPP;
		goto out;
	}

	table->initialized = 1;
	pmu->pe = table->events;
	pmu->pme_count = table->count;

	ret = PFM_SUCCESS;

out:
	if (event_files) {
		for (i = 0; i < nfiles; i++)
			free(event_files[i]);
		free(event_files);
	}

	if (ret != PFM_SUCCESS)
		arm_cmn_free_table(table);

	return ret;
}

static arm_cmn_event_table_t *
arm_cmn_table_from_this(void *this)
{
	pfmlib_pmu_t *pmu = this;
	int instance;

	if (!pmu || !pmu->perf_name)
		return NULL;

	instance = arm_cmn_get_instance_id(pmu->perf_name);
	if (instance < 0 || instance >= ARM_CMN_MAX_INSTANCES)
		return NULL;

	return &arm_cmn_event_tables[instance];
}

static int
arm_cmn_num_mods(const arm_cmn_entry_t *entry)
{
	if (!entry)
		return 0;

	return pfmlib_popcnt(entry->modmsk);
}

static int
arm_cmn_attr2mod(const arm_cmn_entry_t *entry, int attr_idx)
{
	uint64_t modmsk;
	size_t x;
	int n;

	if (!entry)
		return -1;

	modmsk = entry->modmsk;
	n = attr_idx;

	pfmlib_for_each_bit(x, modmsk) {
		if (n == 0)
			return (int)x;
		n--;
	}

	return -1;
}

static int
pfm_arm_detect_cmn(void *this)
{
	pfmlib_pmu_t *pmu = this;
	int instance;

	if (!pmu || !pmu->perf_name)
		return PFM_ERR_NOTSUPP;

	instance = arm_cmn_get_instance_id(pmu->perf_name);
	if (instance < 0)
		return PFM_ERR_NOTSUPP;

	return arm_cmn_load_table(pmu, instance);
}

void
pfm_cmn_unc_terminate(void *this)
{
	arm_cmn_event_table_t *table;

	table = arm_cmn_table_from_this(this);
	if (!table)
		return;

	arm_cmn_free_table(table);
}

int
pfm_cmn_unc_get_event_encoding(void *this, pfmlib_event_desc_t *e)
{
	pfmlib_event_attr_info_t *a;
	arm_cmn_event_table_t *table;
	const arm_cmn_entry_t *event_list;
	const arm_cmn_entry_t *event;
	uint64_t codes[ARM_CMN_CODE_COUNT];
	uint64_t provided_modmsk = 0;
	size_t i;
	int ret;

	table = arm_cmn_table_from_this(this);
	if (!table || !table->initialized)
		return PFM_ERR_NOTSUPP;

	event_list = this_pe(this);
	if (!event_list || e->event < 0 || e->event >= table->count)
		return PFM_ERR_INVAL;

	event = &event_list[e->event];
	memcpy(codes, event->dfl_codes, sizeof(codes));

	for (i = 0; i < (size_t)e->nattrs; i++) {
		int idx;

		a = attr(e, i);
		if (a->ctrl != PFM_ATTR_CTRL_PMU)
			continue;

		if (a->type == PFM_ATTR_UMASK || a->type == PFM_ATTR_RAW_UMASK)
			return PFM_ERR_ATTR;

		idx = (int)a->idx;
		if (idx < 0 || idx >= ARM_CMN_ATTR_MAX)
			return PFM_ERR_ATTR;

		if (!(event->modmsk & ARM_CMN_ATTR_BIT(idx)))
			return PFM_ERR_ATTR;

		ret = arm_cmn_set_field_value(codes, &table->fields[idx], e->attrs[i].ival);
		if (ret != PFM_SUCCESS)
			return ret;

		provided_modmsk |= ARM_CMN_ATTR_BIT(idx);
	}

	e->codes[0] = codes[0];
	e->codes[1] = codes[1];
	e->codes[2] = codes[2];
	e->count = ARM_CMN_CODE_COUNT;

	e->fstr[0] = '\0';
	evt_strcat(e->fstr, "%s", event->name);

	pfmlib_for_each_bit(i, provided_modmsk) {
		uint64_t value;

		if (i >= ARM_CMN_ATTR_MAX)
			continue;

		ret = arm_cmn_get_field_value(codes, &table->fields[i], &value);
		if (ret != PFM_SUCCESS)
			return ret;

		evt_strcat(e->fstr, ":%s=%llu", arm_cmn_mods[i].name,
			   (unsigned long long)value);
	}

	return PFM_SUCCESS;
}

int
pfm_cmn_unc_validate_table(void *this, FILE *fp)
{
	pfmlib_pmu_t *pmu = this;
	const arm_cmn_entry_t *pe = this_pe(this);
	int i;
	int j;
	int error = 0;

	if (!pmu || !pe)
		return PFM_ERR_INVAL;

	for (i = 0; i < pmu->pme_count; i++) {
		if (!pe[i].name) {
			fprintf(fp, "pmu: %s event%d: :: no name\n", pmu->name, i);
			error++;
		}
		if (!pe[i].desc) {
			fprintf(fp, "pmu: %s event%d: %s :: no description\n",
				pmu->name, i, pe[i].name ? pe[i].name : "?");
			error++;
		}
		for (j = i + 1; j < pmu->pme_count; j++) {
			if (pe[i].name && pe[j].name && !strcmp(pe[i].name, pe[j].name)) {
				fprintf(fp, "pmu: %s duplicate event name: %s\n",
					pmu->name, pe[i].name);
				error++;
			}
		}
	}

	return error ? PFM_ERR_INVAL : PFM_SUCCESS;
}

int
pfm_cmn_unc_get_event_info(void *this, int idx, pfm_event_info_t *info)
{
	pfmlib_pmu_t *pmu = this;
	const arm_cmn_entry_t *pe = this_pe(this);

	if (!pmu || !pe || idx < 0 || idx >= pmu->pme_count)
		return PFM_ERR_INVAL;

	info->name = pe[idx].name;
	info->desc = pe[idx].desc;
	info->code = pe[idx].dfl_codes[0];
	info->equiv = NULL;
	info->idx = idx;
	info->pmu = pmu->pmu;
	info->is_precise = 0;
	info->support_hw_smpl = 0;
	info->nattrs = arm_cmn_num_mods(&pe[idx]);

	return PFM_SUCCESS;
}

int
pfm_cmn_unc_get_event_attr_info(void *this, int pidx, int attr_idx,
				pfmlib_event_attr_info_t *info)
{
	pfmlib_pmu_t *pmu = this;
	const arm_cmn_entry_t *pe = this_pe(this);
	int idx;

	if (!pmu || !pe || pidx < 0 || pidx >= pmu->pme_count)
		return PFM_ERR_INVAL;

	if (attr_idx < 0 || attr_idx >= arm_cmn_num_mods(&pe[pidx]))
		return PFM_ERR_INVAL;

	idx = arm_cmn_attr2mod(&pe[pidx], attr_idx);
	if (idx < 0 || idx >= ARM_CMN_ATTR_MAX)
		return PFM_ERR_INVAL;

	info->name = arm_cmn_mods[idx].name;
	info->desc = arm_cmn_mods[idx].desc;
	info->equiv = NULL;
	info->code = idx;
	info->type = arm_cmn_mods[idx].type;
	info->is_dfl = 0;
	info->ctrl = PFM_ATTR_CTRL_PMU;
	info->idx = idx;
	info->dfl_val64 = 0;
	info->is_precise = 0;
	info->support_hw_smpl = 0;

	return PFM_SUCCESS;
}

unsigned int
pfm_cmn_unc_get_event_nattrs(void *this, int pidx)
{
	pfmlib_pmu_t *pmu = this;
	const arm_cmn_entry_t *pe = this_pe(this);

	if (!pmu || !pe || pidx < 0 || pidx >= pmu->pme_count)
		return 0;

	return arm_cmn_num_mods(&pe[pidx]);
}

void
pfm_cmn_unc_perf_validate_pattrs(void *this, pfmlib_event_desc_t *e)
{
	pfmlib_pmu_t *pmu = this;
	int no_smpl = pmu->flags & PFMLIB_PMU_FL_NO_SMPL;
	int i;
	int compact;

	for (i = 0; i < e->npattrs; i++) {
		compact = 0;

		if (e->pattrs[i].type == PFM_ATTR_UMASK)
			continue;

		if (e->pattrs[i].ctrl == PFM_ATTR_CTRL_PERF_EVENT) {
			if (e->pattrs[i].idx == PERF_ATTR_PR)
				compact = 1;

			if (e->pattrs[i].idx == PERF_ATTR_H)
				compact = 1;

			if (no_smpl
			    && (e->pattrs[i].idx == PERF_ATTR_FR
			        || e->pattrs[i].idx == PERF_ATTR_PR
			        || e->pattrs[i].idx == PERF_ATTR_PE))
				compact = 1;

			if (e->pattrs[i].idx == PERF_ATTR_U
			    || e->pattrs[i].idx == PERF_ATTR_K
			    || e->pattrs[i].idx == PERF_ATTR_MG
			    || e->pattrs[i].idx == PERF_ATTR_MH)
				compact = 1;
		}

		if (e->pattrs[i].idx == PERF_ATTR_HWS)
			compact = 1;

		if (compact) {
			pfmlib_compact_pattrs(e, i);
			i--;
		}
	}
}

#define DEFINE_CMN_PMU(n) \
pfmlib_pmu_t arm_cmn_##n##_support={ \
	.desc			= "Arm CMN-600 mesh PMU "#n, \
	.name			= "arm_cmn_"#n, \
	.perf_name		= "arm_cmn_"#n, \
	.pmu			= PFM_PMU_ARM_CMN_##n, \
	.pme_count		= 0, \
	.type			= PFM_PMU_TYPE_UNCORE, \
	.pe			= NULL, \
	.atdesc			= arm_cmn_mods, \
	.pmu_detect		= pfm_arm_detect_cmn, \
	.pmu_terminate		= pfm_cmn_unc_terminate, \
	.max_encoding		= 3, \
	.num_cntrs		= 4, \
	.get_event_encoding[PFM_OS_NONE] = pfm_cmn_unc_get_event_encoding, \
	 PFMLIB_ENCODE_PERF(pfm_cmn_unc_get_perf_encoding), \
	.get_event_first	= pfm_arm_get_event_first, \
	.get_event_next		= pfm_arm_get_event_next, \
	.event_is_valid		= pfm_arm_event_is_valid, \
	.validate_table		= pfm_cmn_unc_validate_table, \
	.get_event_info		= pfm_cmn_unc_get_event_info, \
	.get_event_attr_info	= pfm_cmn_unc_get_event_attr_info, \
	 PFMLIB_VALID_PERF_PATTRS(pfm_cmn_unc_perf_validate_pattrs), \
	.get_event_nattrs	= pfm_cmn_unc_get_event_nattrs, \
};

DEFINE_CMN_PMU(0);
DEFINE_CMN_PMU(1);

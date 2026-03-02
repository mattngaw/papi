#include <sys/types.h>
#include <stdlib.h>

/* private headers */
#include "pfmlib_priv.h"
#include "pfmlib_perf_event_priv.h"
#include "pfmlib_arm_priv.h"
#include "pfmlib_arm_armv8_unc_priv.h"

int
pfm_cmn_unc_get_perf_encoding(void *this, pfmlib_event_desc_t *e)
{
	pfmlib_pmu_t *pmu = this;
	struct perf_event_attr *attr = e->os_data;
	int ret;
	int type;

	if (!pmu->get_event_encoding[PFM_OS_NONE])
		return PFM_ERR_NOTSUPP;

	ret = pmu->get_event_encoding[PFM_OS_NONE](this, e);
	if (ret != PFM_SUCCESS)
		return ret;

	ret = pfm_perf_find_pmu_type(pmu, &type);
	if (ret != PFM_SUCCESS)
		return ret;

	attr->type = type;
	if (e->count > 0)
		attr->config = e->codes[0];
	if (e->count > 1)
		attr->config1 = e->codes[1];
	if (e->count > 2)
		attr->config2 = e->codes[2];

	/* uncore measures at all privilege levels */
	attr->exclude_hv = 0;
	attr->exclude_kernel = 0;
	attr->exclude_user = 0;

	return PFM_SUCCESS;
}

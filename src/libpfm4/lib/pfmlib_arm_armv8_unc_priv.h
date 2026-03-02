#ifndef PFMLIB_ARM_ARMV8_UNC_PRIV_H
#define PFMLIB_ARM_ARMV8_UNC_PRIV_H

#include <sys/types.h>

typedef union {
	uint64_t val;
	struct {
		unsigned long unc_res1:32;	/* reserved */
	} com; /* reserved space for future extensions */
} tx2_unc_data_t;

typedef struct {
	uint64_t val;
} kunpeng_unc_data_t;

typedef struct {
	uint64_t val;
} cmn_unc_data_t;

extern int pfm_tx2_unc_get_perf_encoding(void *this, pfmlib_event_desc_t *e);

//extern int pfm_kunpeng_get_perf_encoding(void *this, pfmlib_event_desc_t *e);

extern int pfm_kunpeng_unc_get_event_encoding(void *this, pfmlib_event_desc_t *e);
extern int pfm_kunpeng_unc_get_perf_encoding(void *this, pfmlib_event_desc_t *e);

extern int pfm_cmn_unc_get_event_encoding(void *this, pfmlib_event_desc_t *e);
extern int pfm_cmn_unc_get_perf_encoding(void *this, pfmlib_event_desc_t *e);
extern int pfm_cmn_unc_validate_table(void *this, FILE *fp);
extern int pfm_cmn_unc_get_event_info(void *this, int idx, pfm_event_info_t *info);
extern int pfm_cmn_unc_get_event_attr_info(void *this, int pidx, int attr_idx, pfmlib_event_attr_info_t *info);
extern unsigned int pfm_cmn_unc_get_event_nattrs(void *this, int pidx);
extern void pfm_cmn_unc_perf_validate_pattrs(void *this, pfmlib_event_desc_t *e);
extern void pfm_cmn_unc_terminate(void *this);

#endif /* PFMLIB_ARM_ARMV8_UNC_PRIV_H */

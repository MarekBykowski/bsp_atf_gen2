/*
 * SPDX-License-Identifier: GPL-2.0
 */

#include <bl_common.h>
#include <axxia_def.h>
#include <arch.h>
#include <arch_helpers.h>
#include <ccn_snoop.h>
#include <debug.h>

static ccn_desc_t axxia_ccn_desc;

static const unsigned char axm5600_is_hw_master_to_rn_id_map[] = {
	9,  /* Cluster 0 */
	19, /* Cluster 1 */
	1,  /* Cluster 2 */
	11  /* Cluster 3 */
};

static const unsigned char axm5600_is_emu_master_to_rn_id_map[] = {
	19, /* Cluster 0 */
	9,  /* Cluster 1 */
	1   /* Cluster 2 */
};

static const unsigned char xlf_a0_is_hw_master_to_rn_id_map[] = {
	11, /* Cluster 0 */
	12, /* Cluster 1 */
	17, /* Cluster 2 */
	18, /* Cluster 3 */
	29, /* Cluster 4 */
	30, /* Cluster 5 */
	35, /* Cluster 6 */
	 0, /* Cluster 7 */
	14, /* Cluster 8 */
	15, /* Cluster 9 */
	32, /* Cluster 10 */
	33  /* Cluster 11 */
};

static const unsigned char xlf_b0_is_hw_master_to_rn_id_map[] = {
	32, /* Cluster 0 */
	29, /* Cluster 1 */
	18, /* Cluster 2 */
	14, /* Cluster 3 */
	12, /* Cluster 4 */
	11, /* Cluster 5 */
	0,  /* Cluster 6 */
	35, /* Cluster 7 */
	33, /* Cluster 8 */
	30, /* Cluster 9 */
	17, /* Cluster 10 */
	15  /* Cluster 11 */
};

static const unsigned char xlf_a0_is_emu_master_to_rn_id_map[] = {
	17, /* Cluster 0 */
	11, /* Cluster 1 */
	29, /* Cluster 2 */
	14, /* Cluster 3 */
	15, /* Cluster 4 */
	32  /* Cluster 5 */
};

/******************************************************************************
 * Helper function to initialize Axxia CCN driver.
 *****************************************************************************/
void plat_axxia_interconnect_init(void)
{
	static const unsigned char *ptr = NULL;
	size_t size = -1;

	if (IS_5600()) {
		axxia_ccn_desc.periphbase = AXM5600_CCN_BASE;
		if (IS_HW() || IS_SIM()) {
			ptr = axm5600_is_hw_master_to_rn_id_map;
			size = ARRAY_SIZE(axm5600_is_hw_master_to_rn_id_map);
		} else if (IS_EMU()) {
			ptr = axm5600_is_emu_master_to_rn_id_map;
			size = ARRAY_SIZE(axm5600_is_emu_master_to_rn_id_map);
		} else {
				ERROR("Unsupported platform\n");
				panic();
		}
	} else if (IS_ANY_6700()) {
		axxia_ccn_desc.periphbase = XLF_CCN_BASE;
		if (IS_SIM() || IS_HW()) {
			if (IS_6700()) { /* Silently it is XLF_A0 */
				ptr = xlf_a0_is_hw_master_to_rn_id_map;
				size = ARRAY_SIZE(xlf_a0_is_hw_master_to_rn_id_map);
			} else if (IS_6700_B0()) {
				ptr = xlf_b0_is_hw_master_to_rn_id_map;
				size = ARRAY_SIZE(xlf_b0_is_hw_master_to_rn_id_map);
			} else {
				ERROR("Unsupported platform\n");
				panic();
			}
		} else if (IS_EMU()) {
			ptr = xlf_a0_is_emu_master_to_rn_id_map;
			size = ARRAY_SIZE(xlf_a0_is_emu_master_to_rn_id_map);
		} else {
			ERROR("Unsupported platform\n");
			panic();
		}
	} else {
		ERROR("Unsupported architecture\n");
		panic();
	}

	axxia_ccn_desc.master_to_rn_id_map = ptr;
	axxia_ccn_desc.num_masters = size;
	ccn_init(&axxia_ccn_desc);
}

/******************************************************************************
 * Helper function to place current master into coherency
 *****************************************************************************/
void plat_axxia_interconnect_enter_coherency(void)
{
	ccn_enter_snoop_dvm_domain(1 << MPIDR_AFFLVL1_VAL(read_mpidr_el1()));
}

/******************************************************************************
 * Helper function to remove current master from coherency
 *****************************************************************************/
void plat_axxia_interconnect_exit_coherency(void)
{
	ccn_exit_snoop_dvm_domain(1 << MPIDR_AFFLVL1_VAL(read_mpidr_el1()));
}

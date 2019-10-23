/*
 *  Copyright (C) 2013 LSI Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <stddef.h>
#include <string.h>
#include <debug.h>
#include <mmio.h>
#include <arch_helpers.h>
#include <ccn_snoop.h>
#include "axxia_def.h"
#include "axxia_private.h"

/*
 * TODO! the ddr_retention_enabled flag should come from
 * u-boot parameter file - how?!?
 */
static int ddr_retention_enabled = 1;
extern void __dead2 axxia_system_reset_wo_sm_56xx(void);
extern void __dead2 axxia_system_reset_wo_sm_xlf(void);

enum {
	AXXIA_ENGINE_CAAL,
	AXXIA_ENGINE_CNAL
};

unsigned long
ncp_caal_regions_acp56xx[] = {
	NCP_REGION_ID(0x0b, 0x05),      /* SPPV2   */
	NCP_REGION_ID(0x0c, 0x05),      /* SED     */
	NCP_REGION_ID(0x0e, 0x05),      /* DPI_HFA */
	NCP_REGION_ID(0x14, 0x05),      /* MTM     */
	NCP_REGION_ID(0x14, 0x0a),      /* MTM2    */
	NCP_REGION_ID(0x15, 0x00),      /* MME     */
	NCP_REGION_ID(0x16, 0x05),      /* NCAV2   */
	NCP_REGION_ID(0x16, 0x10),      /* NCAV22  */
	NCP_REGION_ID(0x17, 0x05),      /* EIOAM1  */
	NCP_REGION_ID(0x19, 0x05),      /* TMGR    */
	NCP_REGION_ID(0x1a, 0x05),      /* MPPY    */
	NCP_REGION_ID(0x1a, 0x23),      /* MPPY2   */
	NCP_REGION_ID(0x1a, 0x21),      /* MPPY3   */
	NCP_REGION_ID(0x1b, 0x05),      /* PIC     */
	NCP_REGION_ID(0x1c, 0x05),      /* PAB     */
	NCP_REGION_ID(0x1f, 0x05),      /* EIOAM0  */
	NCP_REGION_ID(0x31, 0x05),      /* ISB     */
	NCP_REGION_ID(0xff, 0xff)
};

unsigned long
ncp_cnal_regions_acp56xx[] = {
	NCP_REGION_ID(0x28, 0x05),      /* EIOASM0 */
	NCP_REGION_ID(0x29, 0x05),      /* EIOASM1 */
	NCP_REGION_ID(0x2a, 0x05),      /* EIOAS2  */
	NCP_REGION_ID(0x2b, 0x05),      /* EIOAS3  */
	NCP_REGION_ID(0x2c, 0x05),      /* EIOAS4  */
	NCP_REGION_ID(0x2d, 0x05),      /* EIOAS5  */
	NCP_REGION_ID(0x32, 0x05),      /* ISBS    */
	NCP_REGION_ID(0xff, 0xff)
};


unsigned long
ncp_caal_regions_xlf[] =
{
	NCP_REGION_ID(0x0b, 0x05),      /* SPPV2   */
	NCP_REGION_ID(0x0c, 0x05),      /* SED     */
	NCP_REGION_ID(0x0e, 0x05),      /* DPI_HFA */
	NCP_REGION_ID(0x14, 0x05),      /* MTM     */
	NCP_REGION_ID(0x14, 0x0a),      /* MTM2    */
	NCP_REGION_ID(0x15, 0x00),      /* MME     */
	NCP_REGION_ID(0x16, 0x05),      /* NCAV3   */
	NCP_REGION_ID(0x16, 0x10),      /* NCAV32  */
	NCP_REGION_ID(0x17, 0x05),      /* EIOAM   */
	NCP_REGION_ID(0x19, 0x05),      /* TMGR    */
	NCP_REGION_ID(0x1a, 0x05),      /* MPPY    */
	NCP_REGION_ID(0x1a, 0x23),      /* MPPY2   */
	NCP_REGION_ID(0x1a, 0x21),      /* MPPY3   */
	NCP_REGION_ID(0x1b, 0x05),      /* PIC     */
	NCP_REGION_ID(0x1c, 0x05),      /* PAB     */
	NCP_REGION_ID(0x1f, 0x05),      /* EIOAMH  */
	NCP_REGION_ID(0x31, 0x05),      /* ISB     */
	NCP_REGION_ID(0x3e, 0x05),      /* DEC     */
	NCP_REGION_ID(0x40, 0x05),      /* ERIF0   */
	NCP_REGION_ID(0x41, 0x05),      /* ERIF0   */
	NCP_REGION_ID(0x44, 0x05),      /* RIF0    */
	NCP_REGION_ID(0x45, 0x05),      /* RIF0    */
	NCP_REGION_ID(0xff, 0xff)
};

unsigned long
ncp_cnal_regions_xlf[] =
{
	NCP_REGION_ID(0x28, 0x06),      /* EIOASM0 */
	NCP_REGION_ID(0x28, 0x07),      /* EIOASM0 */
	NCP_REGION_ID(0x29, 0x06),      /* EIOASM1 */
	NCP_REGION_ID(0x29, 0x07),      /* EIOASM1 */
	NCP_REGION_ID(0x2a, 0x06),      /* EIOAS2  */
	NCP_REGION_ID(0x2a, 0x07),      /* EIOAS2  */
	NCP_REGION_ID(0x2b, 0x06),      /* EIOAS3  */
	NCP_REGION_ID(0x2b, 0x07),      /* EIOAS3  */
	NCP_REGION_ID(0x32, 0x06),      /* ISBV2s0 */
	NCP_REGION_ID(0x32, 0x07),      /* ISBV2s0 */
	NCP_REGION_ID(0x35, 0x06),      /* ISBV2s1 */
	NCP_REGION_ID(0x35, 0x07),      /* ISBV2s1 */
	NCP_REGION_ID(0xff, 0xff)
};


/*
 * quiesce_vp_engine
 *
 *   quiesce each of the Axxia VP engines by disallowing new memory
 *   transactions and waiting for any outstanding memory transactions
 *   to complete.
 *
 *   If for some reason any outstanding transactions do not complete
 *   within a reasonable time we just give up and continue on to the
 *   next.
 */

static void
quiesce_vp_engine(int engine_type)
{
	unsigned long *engine_regions;
	unsigned long ort_off, owt_off;
	unsigned long *region;
	unsigned ort, owt;
	unsigned short node, target;
	int loop;

	switch (engine_type) {
	case AXXIA_ENGINE_CNAL:
		if (IS_5600()) {
			engine_regions = ncp_cnal_regions_acp56xx;
		} else {
			engine_regions = ncp_cnal_regions_xlf;
		}
		ort_off = 0x1c0;
		owt_off = 0x1c4;
		break;

	case AXXIA_ENGINE_CAAL:
		if (IS_5600()) {
			engine_regions = ncp_caal_regions_acp56xx;
		} else {
			engine_regions = ncp_caal_regions_xlf;
		}
		ort_off = 0xf8;
		owt_off = 0xfc;
		break;

	default:
		return;
	}


		/* for each engine, set read/write transaction limits to zero */
	region = engine_regions;
	while (*region != NCP_REGION_ID(0xff, 0xff)) {
		ncr_write32(*region, 0x8, 0);
		ncr_write32(*region, 0xc, 0);
		region++;
	}


    /* poll until any outstanding request is complete */
	region = engine_regions;
	loop = 0;
	while (*region != NCP_REGION_ID(0xff, 0xff)) {
		node = (*region & 0xffff0000) >> 16;
		target = *region & 0x0000ffff;
		/* read the number of outstanding read/write transactions */
		ncr_read32(*region, ort_off, &ort);
		ncr_read32(*region, owt_off, &owt);

		if ((ort == 0) && (owt == 0)) {
			/* this engine has been quiesced, move on to the next */
			region++;
		} else {
			if (loop++ > 10000) {
				tf_printf(
						"Unable to quiesce region 0x%02x.0x%02x ort=0x%x, owt=0x%x\n",
						node, target, ort, owt);
				region++;
				loop = 0;
				continue;
			}
		}
	}

	return;
}

static void
quiesce_axis(void)
{
    int i;
    int num_regions;
    __uint64_t tzc_base;

    if (IS_5600())
	    tzc_base = TZC_X9_BASE;
    else
	    tzc_base = TZC_XLF_BASE;

    /* h/w defines this as number_of_regions_minus_one */
    num_regions = mmio_read_32(tzc_base) & 0x1f;

    /*
     * for each trustzone region we
     *  1 - clear s_wr_en and s_rd_en in the region_attributes register
     *  2 - clear nsaid_wr_en and nsaid_rd_en in region_id_access register
     */
    for (i = 0; i <= num_regions; i++)
    {
        mmio_write_32(tzc_base + 0x110 + (i * 0x20), 0);
        mmio_write_32(tzc_base + 0x114 + (i * 0x20), 0);
    }
}

/*
  In some situations, resets occasionally (1/500) hang in the
  reset_elm_trace() function during a ddr retention reset.  If a clean
  ELM trace is required, define RESET_ELM_TRACE below.  By default
  RESET_ELM_TRACE is not defined.
*/

#undef RESET_ELM_TRACE
#ifdef RESET_ELM_TRACE

static inline void
reset_elm_trace(void)
{
    int i;
    int num_elms;
    __uint64_t elm_base;
    __uint64_t dbg_ctl;
    __uint64_t cnt_ctl;

    if (IS_5600())
	    num_elms = 2;
    else
	    num_elms = 4;

    for (i = 0; i < num_elms; i++) {
        elm_base = ELM_BASE + (i * 0x00010000);
        dbg_ctl = elm_base + 0x8000;
        cnt_ctl = elm_base + 0x0230;

		/* reset and disable ELM trace */
        mmio_write_32(dbg_ctl, 0x000fff04);

	    /* reset ELM statistics */
        mmio_write_32(cnt_ctl, 1);

		/* re-enable ELM trace */
        mmio_write_32(dbg_ctl, 0x000fff01);
    }
}

#endif	/* RESET_ELM_TRACE */

void
initiate_retention_reset(void)
{
    	int is56xx;

	if (0 == ddr_retention_enabled) {
		tf_printf("DDR Retention Reset is Not Enabled\n");
		return;
	}

	is56xx = IS_5600() ? 1 : 0;

	/* TODO - quiesce VP engines */
	quiesce_vp_engine(AXXIA_ENGINE_CAAL);
	quiesce_vp_engine(AXXIA_ENGINE_CNAL);
	quiesce_axis();

    /* flush all caches */
    plat_flush_dcache_l1();
    dsb();
    plat_flush_dcache_l2();
    dsb();
	/* Flushing l3 is power transition FAM->NOL3->FAM */
	ccn_set_l3_run_mode(CCN_L3_RUN_MODE_NOL3);
	ccn_set_l3_run_mode(CCN_L3_RUN_MODE_FAM);
    dsb();

	/* reset ELM DDR access trace buffer */
#ifdef RESET_ELM_TRACE
	reset_elm_trace();
#endif

	if (is56xx) {
		axxia_system_reset_wo_sm_56xx();
	} else {
		axxia_system_reset_wo_sm_xlf();
	}

	return;
}

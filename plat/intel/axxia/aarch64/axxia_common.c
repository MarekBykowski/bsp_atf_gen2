/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <xlat_tables.h>
#include <axxia_def.h>

/*******************************************************************************
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 ******************************************************************************/
#define DEFINE_CONFIGURE_MMU_EL(_el)				               \
	void configure_mmu_el##_el(unsigned long total_base,	               \
				  unsigned long total_size,	               \
				  unsigned long ro_start,	               \
				  unsigned long ro_limit)	               \
	{							               \
	       mmap_add_region(ro_start, ro_start,		               \
			       ro_limit - ro_start,		               \
			       MT_MEMORY | MT_RO | MT_SECURE);	               \
	       mmap_add_region(ro_limit, ro_limit,                             \
			       TZRAM_SIZE - (ro_limit - ro_start),             \
			       MT_MEMORY | MT_RW | MT_SECURE);	               \
                                                                               \
	       if (IS_6700())                                                  \
		       mmap_add_region(XLF_CCN_BASE, XLF_CCN_BASE,             \
				       XLF_CCN_SIZE,			       \
				       MT_DEVICE | MT_RW | MT_SECURE);         \
                                                                               \
	       mmap_add_region(DEVICE0_BASE, DEVICE0_BASE, DEVICE0_SIZE,       \
			       MT_DEVICE | MT_RW | MT_SECURE);                 \
		                                                               \
	       mmap_add_region(DEVICE1_BASE, DEVICE1_BASE, DEVICE1_SIZE,       \
			       MT_DEVICE | MT_RW | MT_SECURE);                 \
                                                                               \
	       mmap_add_region(DRAM_BASE, DRAM_BASE, DRAM_SIZE,                \
			       MT_MEMORY | MT_RW | MT_SECURE); /*| MT_CACHED);*/     \
		                                                               \
	       init_xlat_tables();				               \
								               \
	       enable_mmu_el##_el(DISABLE_DCACHE);		               \
	}

/* Define EL1 and EL3 variants of the function initialising the MMU */
DEFINE_CONFIGURE_MMU_EL(1)
DEFINE_CONFIGURE_MMU_EL(3)

unsigned long plat_get_ns_image_entrypoint(void)
{
	return NS_IMAGE_OFFSET;
}

uint64_t plat_get_syscnt_freq(void)
{
	static uint64_t cntfrq = 0;

	/*
	  This gets set by the previous stage -- don't change it!
	*/

	if (0 == plat_my_core_pos() && 0 == cntfrq)
		__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r" (cntfrq));

	return cntfrq;
}

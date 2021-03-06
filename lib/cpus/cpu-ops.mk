#
# Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Cortex A57 specific optimisation to skip L1 cache flush when
# cluster is powered down.
SKIP_A57_L1_FLUSH_PWR_DWN	?=0

# Process SKIP_A57_L1_FLUSH_PWR_DWN flag
$(eval $(call assert_boolean,SKIP_A57_L1_FLUSH_PWR_DWN))
$(eval $(call add_define,SKIP_A57_L1_FLUSH_PWR_DWN))


# CPU Errata Build flags. These should be enabled by the
# platform if the errata needs to be applied.

# Flag to apply errata 826319 during reset. This errata applies only to
# revision <= r0p2 of the Cortex A53 cpu.
ERRATA_A53_826319	?=0

# Flag to apply errata 836870 during reset. This errata applies only to
# revision <= r0p3 of the Cortex A53 cpu. From r0p4 and onwards, this
# errata is enabled by default.
ERRATA_A53_836870	?=0

# Flag to apply errata 806969 during reset. This errata applies only to
# revision r0p0 of the Cortex A57 cpu.
ERRATA_A57_806969	?=0

# Flag to apply errata 813420 during reset. This errata applies only to
# revision r0p0 of the Cortex A57 cpu.
ERRATA_A57_813420	?=0

# Process ERRATA_A53_826319 flag
$(eval $(call assert_boolean,ERRATA_A53_826319))
$(eval $(call add_define,ERRATA_A53_826319))

# Process ERRATA_A53_836870 flag
$(eval $(call assert_boolean,ERRATA_A53_836870))
$(eval $(call add_define,ERRATA_A53_836870))

# Process ERRATA_A57_806969 flag
$(eval $(call assert_boolean,ERRATA_A57_806969))
$(eval $(call add_define,ERRATA_A57_806969))

# Process ERRATA_A57_813420 flag
$(eval $(call assert_boolean,ERRATA_A57_813420))
$(eval $(call add_define,ERRATA_A57_813420))

# Process WORKAROUND_CVE_2017_5715 flag
# Invalidate the Branch Target Buffer (BTB) on entry to EL3 by disabling
# and enabling the MMU
WORKAROUND_CVE_2017_5715	?=0
$(eval $(call assert_boolean,WORKAROUND_CVE_2017_5715))
$(eval $(call add_define,WORKAROUND_CVE_2017_5715))

# Process WORKAROUND_CVE_2018_3639 flag
# Set the "Disable Load Pass Store" bit in the ACTLR_EL1 for all cores
WORKAROUND_CVE_2018_3639	?=0
$(eval $(call assert_boolean,WORKAROUND_CVE_2018_3639))
$(eval $(call add_define,WORKAROUND_CVE_2018_3639))

# Process WORKAROUND_CVE_2017_7564 flag
# Disable secure self-hosted debug via MDCR_EL3/SDCR
WORKAROUND_CVE_2017_7564   ?=0
$(eval $(call assert_boolean,WORKAROUND_CVE_2017_7564))
$(eval $(call add_define,WORKAROUND_CVE_2017_7564))

# Process WORKAROUND_CVE_2017_7563 flag
# RO memory is always executable at AArch64 Secure EL1
WORKAROUND_CVE_2017_7563   ?=0
$(eval $(call assert_boolean,WORKAROUND_CVE_2017_7563))
$(eval $(call add_define,WORKAROUND_CVE_2017_7563))

# Process WORKAROUND_CVE_2017_15031 flag
# Not initializing or saving/restoring PMCR_EL0 can leak secure world
# timing information
WORKAROUND_CVE_2017_15031  ?=0
$(eval $(call assert_boolean,WORKAROUND_CVE_2017_15031))
$(eval $(call add_define,WORKAROUND_CVE_2017_15031))

# Build flag to include AArch32 registers in cpu context save and restore
# during world switch. This flag must be set to 0 for AArch64-only platforms.
CTX_INCLUDE_AARCH32_REGS    ?=0
$(eval $(call assert_boolean,CTX_INCLUDE_AARCH32_REGS))
$(eval $(call add_define,CTX_INCLUDE_AARCH32_REGS))

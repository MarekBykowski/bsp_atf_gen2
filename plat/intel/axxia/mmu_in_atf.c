
 /*
  * TCR flags.
  */
#define VA_BITS         (42)    /* 42 bits virtual address */
#define TCR_T0SZ(x)     ((64 - (x)) << 0)
#define TCR_IRGN_NC     (0 << 8)
#define TCR_IRGN_WBWA       (1 << 8)
#define TCR_IRGN_WT     (2 << 8)
#define TCR_IRGN_WBNWA      (3 << 8)
#define TCR_IRGN_MASK       (3 << 8)
#define TCR_ORGN_NC     (0 << 10)
#define TCR_ORGN_WBWA       (1 << 10)
#define TCR_ORGN_WT     (2 << 10)
#define TCR_ORGN_WBNWA      (3 << 10)
#define TCR_ORGN_MASK       (3 << 10)
#define TCR_SHARED_NON      (0 << 12)
#define TCR_SHARED_OUTER    (2 << 12)
#define TCR_SHARED_INNER    (3 << 12)
#define TCR_TG0_4K      (0 << 14)
#define TCR_TG0_64K     (1 << 14)
#define TCR_TG0_16K     (2 << 14)
#define TCR_EL1_IPS_BITS    (UL(3) << 32)   /* 42 bits physical address */
#define TCR_EL2_IPS_BITS    (3 << 16)   /* 42 bits physical address */
#define TCR_EL3_IPS_BITS    (3 << 16)   /* 42 bits physical address */

/* PTWs cacheable, inner/outer WBWA and inner shareable */
#define TCR_FLAGS       (TCR_TG0_64K |      \
                TCR_SHARED_INNER |  \
                TCR_ORGN_WBWA |     \
                TCR_IRGN_WBWA |     \
                TCR_T0SZ(VA_BITS))


#define TCR_EL1_RSVD        (1 << 31)
#define TCR_EL2_RSVD        (1 << 31 | 1 << 23)
#define TCR_EL3_RSVD        (1 << 31 | 1 << 23)
#define TCR_EL2_IPS_BITS    (3 << 16)   /* 42 bits physical address */

/*                
 * Memory types
 */                 
#ifdef __ASSEMBLY__
#define _AC(X, Y)   X
#else
#define _AC(X, Y)   (X##Y)
#endif

#define UL(x)       _AC(x, UL)

#define MT_DEVICE_NGNRNE    0
#define MT_DEVICE_NGNRE     1
#define MT_DEVICE_GRE       2
#define MT_NORMAL_NC        3
#define MT_NORMAL       4

#define MEMORY_ATTRIBUTES   ((0x00 << (MT_DEVICE_NGNRNE*8)) |   \
                (0x04 << (MT_DEVICE_NGNRE*8)) |     \
                (0x0c << (MT_DEVICE_GRE*8)) |       \
                (0x44 << (MT_NORMAL_NC*8)) |        \
                (UL(0xff) << (MT_NORMAL*8)))

#define CR_M        (1 << 0)    /* MMU enable           */
#define CR_A        (1 << 1)    /* Alignment abort enable   */
#define CR_C        (1 << 2)    /* Dcache enable        */
#define CR_SA       (1 << 3)    /* Stack Alignment Check Enable */
#define CR_I        (1 << 12)   /* Icache enable        */
#define CR_WXN      (1 << 19)   /* Write Permision Imply XN */
#define CR_EE       (1 << 25)   /* Exception (Big) Endian   */

/* AttrIndx[2:0] */
#define PMD_ATTRINDX(t)     ((t) << 2)

/*
 *  * Hardware page table definitions. 
 *   *
 *    * Level 2 descriptor (PMD).
 *     */   
#define PMD_TYPE_MASK       (3 << 0)
#define PMD_TYPE_FAULT      (0 << 0)
#define PMD_TYPE_TABLE      (3 << 0)
#define PMD_TYPE_SECT       (1 << 0)

/*    
 *     * Section
 *      */
#define PMD_SECT_NON_SHARE  (0 << 8)
#define PMD_SECT_OUTER_SHARE    (2 << 8)
#define PMD_SECT_INNER_SHARE    (3 << 8)
#define PMD_SECT_AF     (1 << 10)
#define PMD_SECT_NG     (1 << 11)
#define PMD_SECT_PXN        (UL(1) << 53)
#define PMD_SECT_UXN        (UL(1) << 54)

#define PGTABLE_SIZE    (0x10000)
#define SECTION_SHIFT 29

static inline void set_ttbr_tcr_mair(int el, uint64_t table, uint64_t tcr, uint64_t attr)
{       
    __asm__ __volatile__("dsb sy");
    if (el == 1) {
        __asm__ __volatile__("msr ttbr0_el1, %0" : : "r" (table) : "memory");
        __asm__ __volatile__("msr tcr_el1, %0" : : "r" (tcr) : "memory");
        __asm__ __volatile__("msr mair_el1, %0" : : "r" (attr) : "memory");
    } else if (el == 2) {
        __asm__ __volatile__("msr ttbr0_el2, %0" : : "r" (table) : "memory");
        __asm__ __volatile__("msr tcr_el2, %0" : : "r" (tcr) : "memory");
        __asm__ __volatile__("msr mair_el2, %0" : : "r" (attr) : "memory");
    } else if (el == 3) {
        __asm__ __volatile__("msr ttbr0_el3, %0" : : "r" (table) : "memory");
        __asm__ __volatile__("msr tcr_el3, %0" : : "r" (tcr) : "memory");
        __asm__ __volatile__("msr mair_el3, %0" : : "r" (attr) : "memory");
    }   
    __asm__ __volatile__("isb");
}                 

static inline unsigned int get_sctlr(void)
{           
    unsigned int el, val;
        
    el = 3; /*current_el();*/
    if (el == 1)
        __asm__ __volatile__("mrs %0, sctlr_el1" : "=r" (val) : : "cc");
    else if (el == 2)
        __asm__ __volatile__("mrs %0, sctlr_el2" : "=r" (val) : : "cc");
    else
        __asm__ __volatile__("mrs %0, sctlr_el3" : "=r" (val) : : "cc");
                  
    return val;
}       
                  
static inline void set_sctlr(unsigned int val)
{   
    unsigned int el;
                  
    el = 3; /*current_el();*/
    if (el == 1)
        __asm__ __volatile__("msr sctlr_el1, %0" : : "r" (val) : "cc");
    else if (el == 2)
        __asm__ __volatile__("msr sctlr_el2, %0" : : "r" (val) : "cc");
    else
        __asm__ __volatile__("msr sctlr_el3, %0" : : "r" (val) : "cc");

    __asm__ __volatile__("isb");
}


static
void set_pgtable_section(uint64_t *page_table, uint64_t index, uint64_t section,
             uint64_t memory_type, uint64_t share)
{
    uint64_t value;
    value = section | PMD_TYPE_SECT | PMD_SECT_INNER_SHARE | PMD_SECT_AF;
    value |= PMD_ATTRINDX(memory_type);
    value |= share;
    /*value |= SET_NS;*/
    page_table[index] = value;
}


static 
void setup_pt(void)
{
     uint64_t *ttbr0 = (uint64_t*) 0x8031030000;
     uint64_t ddr_start = 0, lsm_start = 0x8020000000;
     uint64_t ddr_end = 0x40000000, lsm_end = lsm_start + 0x20000000, i, j;

     /* Setup an identity-mapping for all spaces */
     for (i = 0; i < (PGTABLE_SIZE >> 3); i++) {
         set_pgtable_section((void*)ttbr0, i, i << SECTION_SHIFT,
                      MT_DEVICE_NGNRNE, PMD_SECT_NON_SHARE);
                     /*MT_DEVICE_NGNRNE, PMD_SECT_OUTER_SHARE);*/
     }

     /* Setup an identity-mapping for all RAM space */
     for (j = ddr_start >> SECTION_SHIFT;
          j < ddr_end >> SECTION_SHIFT; j++) {
         set_pgtable_section((void*)ttbr0, j, j << SECTION_SHIFT,
                     MT_NORMAL, PMD_SECT_NON_SHARE);
     }

     for (j = lsm_start >> SECTION_SHIFT;
          j < lsm_end >> SECTION_SHIFT; j++) {
            printf("doing LSM 0x%lx 0x%lx\n", j, j<< SECTION_SHIFT);
         set_pgtable_section((void*)ttbr0, j, j << SECTION_SHIFT,
                     MT_NORMAL, PMD_SECT_NON_SHARE);
     }
}

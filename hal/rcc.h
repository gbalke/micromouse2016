#pragma once
#include <stdint.h>

struct rcc_register {
    uint32_t cr;
    uint32_t pllcfgr;
    uint32_t cfgr;
    uint32_t cir;
    uint32_t ahb1rstr;
    uint32_t ahb2rstr;
    uint32_t reserved[2];
    uint32_t apb1rstr;
    uint32_t apb2rstr;
    uint32_t reserved2[2];
    uint32_t ahb1enr;
    uint32_t ahb2enr;
    uint32_t reserved3[2];
    uint32_t apb1enr;
    uint32_t apb2enr;
    uint32_t reserved4[2];
    uint32_t ahb1lpenr;
    uint32_t ahb2lpenr;
    uint32_t reserved5[2];
    uint32_t apb1lpenr;
    uint32_t apb2lpenr;
    uint32_t reserved6[2];
    uint32_t bdcr;
    uint32_t csr;
    uint32_t reserved7[2];
    uint32_t sscgr;
    uint32_t plli2scfgr;
    uint32_t reserved8;
    uint32_t dckcfgr;
};

static volatile rcc_register *const rcc = (rcc_register *const) 0x40023800;

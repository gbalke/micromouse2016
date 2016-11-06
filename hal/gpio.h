#pragma once
#include <cstdint>

struct gpio_port {
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t idr;
    uint32_t odr;
    uint32_t bsrr;
    uint32_t lckr;
    uint32_t afrl;
    uint32_t afrh;
    // needed to properly align ports
    uint32_t reserved[246];
};

static volatile gpio_port *const gpio_base = (gpio_port *const) 0x40020000;

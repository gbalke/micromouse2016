// TODO: add entry to NVIC vector table
#include "PinNames.h"
#include "digital_input.h"
#include "interrupt_pin.h"

static void (*volatile *const ivt)() = (void (**)()) 0x00000040;

struct nvic_register {
    // Some of the higher register values are reserved
    uint32_t iser[32];
    uint32_t icer[32];
    uint32_t ispr[32];
    uint32_t icpr[32];
    uint32_t iabr[32];
    uint32_t reserved[32];
    uint32_t ipr[124];
};

static volatile nvic_register *const nvic = (nvic_register *const) 0xE000E100;

struct exti_register {
    uint32_t imr;
    uint32_t emr;
    uint32_t rtsr;
    uint32_t ftsr;
    uint32_t swier;
    uint32_t pr;
};

static volatile exti_register *const exti = (exti_register *const) 0x40013C00;

struct syscfg_register {
    uint32_t memrmp;
    uint32_t pmc;
    uint32_t exticr[4];
    uint32_t cmpcr;
};

static volatile syscfg_register *const syscfg = (syscfg_register *const) 0x40013800;

struct interrupt_handler {
    void *callback;
    void *data;
};

static interrupt_handler exti_rising_handlers[16] = {0};
static interrupt_handler exti_falling_handlers[16] = {0};
static int exti9_5_pin = 0;
static int exti15_10_pin = 0;

static inline void main_handler(uint8_t pin)
{
    int index = pin / 4;
    int offset = pin % 4;
    // Finds which actual pin was triggered
    uint8_t port = (syscfg->exticr[index] >> 4 * offset) & 0xF;
    interrupt_handler handler;
    if(DigitalInput::read_pin(port, pin)) {
        handler = exti_rising_handlers[pin];
    } else {
        handler = exti_falling_handlers[pin];
    }
    if(handler.callback && handler.data) {
        void (*callback)(void *data) = (void (*)(void *)) handler.callback;
        callback(handler.data);
    } else if(handler.callback) {
        void (*callback)() = (void (*)()) handler.callback;
        callback();
    }
    // Clears interrupt
    exti->pr |= (0x1 << pin);
}

// Handlers must have these exact names to overwrite vector table entries
extern "C" {

__attribute__((interrupt))
void EXTI0_IRQHandler() { main_handler(0); }
__attribute__((interrupt))
void EXTI1_IRQHandler() { main_handler(1); }
__attribute__((interrupt))
void EXTI2_IRQHandler() { main_handler(2); }
__attribute__((interrupt))
void EXTI3_IRQHandler() { main_handler(3); }
__attribute__((interrupt))
void EXTI4_IRQHandler() { main_handler(4); }
__attribute__((interrupt))
void EXTI9_5_IRQHandler() { main_handler(exti9_5_pin); }
__attribute__((interrupt))
void EXTI15_10_IRQHandler() { main_handler(exti15_10_pin); }

}

InterruptPin::InterruptPin(PinName pin)
: DigitalInput(pin)
{
}

InterruptPin::~InterruptPin()
{
    unregister(BOTH);
}

void InterruptPin::register_edge(Edge edge, void (*callback)())
{
    enable_exti(edge);
    if(edge == RISING || edge == BOTH) {
        exti_rising_handlers[this->pin] = {(void *)callback, nullptr};
    }
    if(edge == FALLING || edge == BOTH) {
        exti_falling_handlers[this->pin] = {(void *)callback, nullptr};
    }
}

void InterruptPin::register_edge(Edge edge, void (*callback)(void *), void *data)
{
    enable_exti(edge);
    if(edge == RISING || edge == BOTH) {
        exti_rising_handlers[this->pin] = {(void *)callback, data};
    }
    if(edge == FALLING || edge == BOTH) {
        exti_falling_handlers[this->pin] = {(void *)callback, data};
    }
}

void InterruptPin::enable_exti(Edge edge)
{
    // unmasks interrupt
    exti->imr |= 0x1 << this->pin;
    if(edge == RISING || edge == BOTH) {
        // enables rising edge
        exti->rtsr |= 0x1 << this->pin;
    }
    if(edge == FALLING || edge == BOTH) {
        // enables falling edge
        exti->ftsr |= 0x1 << this->pin;
    }
    int index = this->pin / 4;
    int pos = this->pin % 4;
    uint32_t exticr = syscfg->exticr[index];
    // selects correct port for EXTI interrupt
    exticr &= ~(0xF << 4 * pos);
    exticr |= this->port_offset << 4 * pos;
    syscfg->exticr[index] = exticr;
    // 9-5 and 10-15 share an interrupt handler, so we must record which is active
    if(this->pin <= 9 && this->pin >= 5) {
        exti9_5_pin = this->pin;
    } else if(this->pin <= 15 && this->pin >= 10) {
        exti15_10_pin = this->pin;
    }
    // Enable the interrupt in NVIC
    switch(pin) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            nvic->iser[0] |= 0x1 << (pin + 6);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            nvic->iser[0] |= 0x1 << 23;
            break;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            nvic->iser[1] |= 0x1 << 8;
            break;
    }
}

void InterruptPin::unregister(Edge edge)
{
    if(edge == RISING || edge == BOTH) {
        exti->rtsr &= ~(0x1 << this->pin);
        exti_rising_handlers[this->pin] = {0};
    }
    if(edge == FALLING || edge == BOTH) {
        exti->ftsr &= ~(0x1 << this->pin);
        exti_falling_handlers[this->pin] = {0};
    }
    // Disable interrupt in NVIC
    switch(pin) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            nvic->icer[0] |= 0x1 << (pin + 6);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            nvic->icer[0] |= 0x1 << 23;
            break;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            nvic->icer[1] |= 0x1 << 8;
            break;
    }
}

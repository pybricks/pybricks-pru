#include <stdint.h>

#include "am1808.h"

// Shared RAM (control input)
typedef struct shared_ram {
    // Declare this as a u32 to force more-efficient codegen
    uint32_t pwms;
} shared_ram;
// XXX: The real address in use here is 0x80010000
// There is currently a compiler bug where ctable entries with the MSB set
// do not get optimized correctly. We lie to the PRU compiler here, but the
// hardware indeed contains the correct address we *actually* want.
// Upstream bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=121124
// Revisit this once the patch lands in a release.
static volatile shared_ram * const SHARED = (volatile shared_ram *)0x7f010000;
#pragma ctable_entry 30 0x7f010000

// LED definitions
// LEDx corresponds to DIODEx in the EV3 schematics
// LEDx defined as n ==> pin is PRU1_R30[n]
#define LED0    12
#define LED1    10
#define LED2    13
#define LED3    11

static inline void update_pwm(uint8_t val, uint8_t time_now, uint32_t gpio_bit) {
    // We want to force generation of the optimized set/clr opcodes
    if (time_now < val) {
        asm volatile("set r30, r30, %0"::"I"(gpio_bit));
    } else {
        asm volatile("clr r30, r30, %0"::"I"(gpio_bit));
    }
}

void main() {
    uint32_t pwms = 0;
    while (1) {
        // 24 MHz / 256 ==> 93.75 kHz tick rate for this counter
        uint8_t time_now = (*TIMER0_TIM34) >> 8;

        if (time_now == 0) {
            // 24 MHz / 256 / 256 ~= 366 Hz update rate
            pwms = SHARED->pwms;
        }

        update_pwm(pwms >> 0, time_now, LED0);
        update_pwm(pwms >> 8, time_now, LED1);
        // Intentional swap of 2 and 3
        // This puts the pins into the order R, G, R, G
        update_pwm(pwms >> 16, time_now, LED3);
        update_pwm(pwms >> 24, time_now, LED2);
    }
}

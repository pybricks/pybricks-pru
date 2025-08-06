#include <stdint.h>

#include "am1808.h"
#include "rproc_ev3_pru1.h"

// XXX: The real address in use here is 0x80010000
// There is currently a compiler bug where ctable entries with the MSB set
// do not get optimized correctly. We lie to the PRU compiler here, but the
// hardware indeed contains the correct address we *actually* want.
// Upstream bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=121124
// Revisit this once the patch lands in a release.
static volatile pbdrv_rproc_ev3_pru1_shared_ram_t * const SHARED_HACK = (volatile pbdrv_rproc_ev3_pru1_shared_ram_t *)0x7f010000;
static volatile pbdrv_rproc_ev3_pru1_shared_ram_t * const SHARED = (volatile pbdrv_rproc_ev3_pru1_shared_ram_t *)0x80010000;
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

// I2C-related code

// Sensor port pin definitions for software I2C driver.
// It just so happens that all relevant SCL/SDA pins are on GPIO banks 0 or 1
// which both exist in the same hardware register (bank 0 at bits[15:0] and
// bank 1 at bits[31:16])
// We therefore use a simplified pin definition and specify only the bit index
// within that register. The PRU has efficient bit set/clear opcodes
// that work well with this.
#define SENSOR_PORT_1_PIN_SCL   2
#define SENSOR_PORT_1_PIN_SDA   15
#define SENSOR_PORT_2_PIN_SCL   14
#define SENSOR_PORT_2_PIN_SDA   13
#define SENSOR_PORT_3_PIN_SCL   12
#define SENSOR_PORT_3_PIN_SDA   (14 + 16)
#define SENSOR_PORT_4_PIN_SCL   1
#define SENSOR_PORT_4_PIN_SDA   (15 + 16)

// This timeout specifies the *total permissible delay* of an I2C transaction.
// This is the permissible sum total of all clock stretching as well as
// the initial wait for the bus to become free. Time spent actually transferring
// data is not deducted from this timeout.
#define I2C_TIMEOUT_MS      500
#define I2C_TIMEOUT_LOOPS   (I2C_TIMEOUT_MS * 2 * PBDRV_RPROC_EV3_PRU1_I2C_CLK_SPEED_HZ / 1000)

// This I2C controller operates on the level of complete transactions
// (called "data transfer formats" in the Phillips/NXP specification).
// The following transaction types are supported:
// - writing data
// - reading data
// - a combined transfer of one write followed by one read
//
// In a write transaction, the device address is sent followed by the
// controller continuing to send 0 or more additional bytes.
// (A transfer with 0 additional bytes can be used to probe for the
// presence of a device at the specified address.)
//
// In a read transaction, the device address is sent by the controller
// followed by the bus changing directions. The controller continues
// to output clock pulses and the device sends data bytes.
//
// In a combined transaction, a "write" transfer is first sent.
// Instead of sending a "stop" to indicate the end of the transaction,
// a "repeated start" is sent, followed by a "read" transfer.
// This controller only implements this one specific type of combined
// transaction, which is the most common type used by devices.
//
// As a quirk to handle NXT sensors such as the ultrasonic sensor,
// the "repeated start" bus signal can be replaced with a "stop",
// additional SCL clock pulse, and a "start". This is needed
// as a workaround for firmware bugs in the sensor device.
// This looks like the following on the bus:
// SCL __/¯¯¯¯¯\_____/¯¯¯¯¯\_____/¯¯¯¯¯\__
// SDA _<  ACK  >_______/¯¯¯¯¯¯¯¯¯¯¯\_____
//                  ^----^^-------^^----^
//                   stop   pulse   start
//
// For comparison, a normal repeated start looks like the following:
// SCL __/¯¯¯¯¯\_____/¯¯¯¯¯\__
// SDA _<  ACK  >¯¯¯¯¯¯¯\_____
//                   ^ ^----^
//                   |   |
//                   |   +- repeated start
//                   +- *not* a stop condition

// This I2C controller supports single-master operation only
// and does not support bus arbitration. It does not support
// operation as a peripheral. This means that many devices can
// be connected to the bus, but it is not supported to connect
// e.g. two EV3s together.
//
// This controller does support clock stretching. This means that
// a peripheral which cannot keep up with the controller can pull
// SCL low to slow down the controller.
//
// This controller does not support 10-bit addressing or reserved
// addresses. All addresses are considered valid and 7 bits long.
// This is also required by certain LEGO peripherals.

// This controller is implemented as a finite state machine.
// The overall state transition diagram is as follows:
//      +-> IDLE
//      |     |
//      |     |
//      |     |
//      |     V
//      |   START
//      |     |
//      |     |
//      |     |
//      |     V
//      |   TX_ADDR
//      |     |
//      ^     |
//      |     |
//      |     V         (empty ping)
//      |   R_ACK_ADDR >------------+
//      |     |                     |
//      |     |                     |
//      |     |                     |
//      |     V                     |
//      |   TX_BYTE <-+             |
//      |     |       |             |
//      ^     |       |             |
//      |     |       |             V
//      |     V       |             |
//      |   R_ACK >---+->-------+->-+
//      |     |                 |   |
//      |     |                 V   V
//      |     |                 |   |
//      |     V                 |   |
//      |   STOP (NXT quirk)    |   |
//      |     |                 |   |
//      |     |                 |   |
//      |     |                 |   |
//      |     V                 |   |
//      ^   REP_START <---------+   V
//      |     |                     |
//      |     |                     |
//      |     |                     |
//      |     V                     |
//      |   START                   |
//      |     |                     |
//      |     |                     |
//      |     |                     |
//      |     V                     |
//      |   TX_ADDR                 |
//      |     |                     |
//      ^     |                     V
//      |     |                     |
//      |     V                     |
//      |   R_ACK_ADDR              |
//      |     |                     |
//      |     |                     |
//      |     |                     |
//      |     V                     |
//      |   RX_BYTE <-+             |
//      |     |       |             |
//      ^     |       |             V
//      |     |       |             |
//      |     V       |             |
//      |   W_ACK ----+             |
//      |     |                     |
//      |     |                     |
//      |     |                     |
//      |     V                     |
//      +-< STOP <------------------+
//
// Most states have at least two phases, one for SCL low and
// one for SCL high. In general, state transitions are usually made
// while SCL is high. If SCL is stretched low by the peripheral,
// an additional clock stretching phase will occur.

enum {
    I2C_STATE_IDLE,
    I2C_STATE_START,
    I2C_STATE_TX_BYTE,
    I2C_STATE_TX_ADDR,
    I2C_STATE_R_ACK,
    I2C_STATE_R_ACK_ADDR,
    I2C_STATE_STOP,
    I2C_STATE_RX_BYTE,
    I2C_STATE_W_ACK,
    I2C_STATE_REP_START,
};

// Each instance of a software I2C controller has its state stored in
// this struct (which lives in PRU local RAM). The main function
// invokes the handler function once per state per timer transition.

typedef struct i2c_state_struct {
    // The following fields are mostly an unpacked version of the
    // shared memory control structure.
    uint8_t *buffer;
    uint32_t timeout;
    union {
        // This layout matches pbdrv_rproc_ev3_pru1_i2c_command_t.flags
        struct {
            uint8_t flags;
            uint8_t wlen;
            uint8_t rlen;
            uint8_t addr;
        };
        uint32_t control_params;
    };

    // Current FSM state (using the I2C_STATE_* enum values)
    uint8_t state;
    // Clock phase within the current state
    uint8_t phase;
    // Temporary holding value for the byte currently being exchanged
    uint8_t work_byte;
    // Bit index currently being exchanged
    uint8_t work_bit;

    // Bit index of the IO pins in GPIO bank 0/1
    uint8_t scl_bit;
    uint8_t sda_bit;
} i2c_state_struct;
i2c_state_struct i2c_states[PBDRV_RPROC_EV3_PRU1_NUM_I2C_BUSES];

// Because the PRU GCC compiler seems to struggle significantly with
// optimizing bit set/clear/test opcodes, we use the following functions
// to force the code generation we want.

static inline __attribute__((always_inline)) void i2c_low(uint8_t gpio_bit) {
    uint32_t r = GPIO_BANK_01->dir;
    asm("clr %0, %0, %1" : "+r"(r) : "r"(gpio_bit));
    GPIO_BANK_01->dir = r;
}
static inline __attribute__((always_inline)) void i2c_hi(uint8_t gpio_bit) {
    uint32_t r = GPIO_BANK_01->dir;
    asm("set %0, %0, %1" : "+r"(r) : "r"(gpio_bit));
    GPIO_BANK_01->dir = r;
}

// Test if the device is attempting to stretch the clock.
// If it is, jump to the `handle_timeout` label.
#define I2C_HANDLE_CLOCK_STRETCH()                      \
    asm goto(                                           \
        "qbbc %l[handle_timeout], %0, %1"               \
        :: "r"(GPIO_BANK_01->in_data), "r"(scl_bit)     \
        :: handle_timeout                               \
    )

// If SDA is high (the bit is set), it is a NAK.
// Jump to the `handle_nak` label if so.
#define I2C_CHECK_NAK()                                 \
    asm goto(                                           \
        "qbbs %l[handle_nak], %0, %1"                   \
        :: "r"(GPIO_BANK_01->in_data), "r"(sda_bit)     \
        :: handle_nak                                   \
    )

// Wait for both SCL and SDA to be high, or else the bus is busy.
// If the bus is busy, jump to the `handle_timeout` label.
#define I2C_CHECK_BUS_BUSY()                                        \
    asm goto(                                                       \
        "qbbc %l[handle_timeout], %0, %1\n"                         \
        "qbbc %l[handle_timeout], %0, %2\n"                         \
        :: "r"(GPIO_BANK_01->in_data), "r"(scl_bit), "r"(sda_bit)   \
        :: handle_timeout                                           \
    )

// Modify the gpio value as appropriate depending on the current bit
// (i.e. work_bit in work_byte)
#define I2C_SDA_SET_OUTPUT_BIT(gpio)                    \
    asm(                                                \
        "qbbs 1f, %1, %2\n"                             \
        "clr %0, %0, %3\n"                              \
        "jmp 2f\n"                                      \
        "1:\n"                                          \
        "set %0, %0, %3\n"                              \
        "2:"                                            \
        : "+r"(gpio)                                    \
        : "r"(work_byte), "r"(work_bit), "r"(sda_bit)   \
    )

// Test SDA and set the current bit in work_byte if it is high.
#define I2C_SDA_GET_INPUT_BIT()                                     \
    asm(                                                            \
        "qbbc 1f, %1, %2\n"                                         \
        "set %0, %0, %3\n"                                          \
        "1:"                                                        \
        : "+r"(work_byte)                                           \
        : "r"(GPIO_BANK_01->in_data), "r"(sda_bit), "r"(work_bit)   \
    )

static void handle_i2c(volatile pbdrv_rproc_ev3_pru1_i2c_command_t *cmd, i2c_state_struct *i2c) {
    // Load most of the relevant state ahead of time to improve codegen
    uint8_t state = i2c->state;
    uint8_t phase = i2c->phase;
    uint8_t work_byte = i2c->work_byte;
    uint8_t work_bit = i2c->work_bit;

    uint8_t scl_bit = i2c->scl_bit;
    uint8_t sda_bit = i2c->sda_bit;

    switch (state) {
        case I2C_STATE_IDLE: {
            uint32_t flags = cmd->flags;
            if ((flags & (PBDRV_RPROC_EV3_PRU1_I2C_CMD_START | PBDRV_RPROC_EV3_PRU1_I2C_STAT_DONE)) != PBDRV_RPROC_EV3_PRU1_I2C_CMD_START) {
                break;
            }

            i2c->buffer = (uint8_t *)cmd->buffer;
            i2c->control_params = flags;

            state = I2C_STATE_START;
            phase = 0;
            i2c->timeout = I2C_TIMEOUT_LOOPS;

            cmd->flags = 0;

            __attribute__((fallthrough));
        }

        case I2C_STATE_START:
            // SCL ¯¯¯
            // SDA ¯\_

            if (phase == 0) {
                I2C_CHECK_BUS_BUSY();
                phase = 1;
            }

            // Intentionally no else, since the moment the bus *is* idle
            // we can proceed to starting a transaction.
            if (phase == 1) {
                i2c_low(sda_bit);

                state = I2C_STATE_TX_ADDR;
                phase = 0;
                work_bit = 7;
                if (i2c->wlen == 0 && i2c->rlen > 0) {
                    work_byte = (i2c->addr << 1) | 1;
                } else {
                    work_byte = i2c->addr << 1;
                }
            }
            break;

        case I2C_STATE_TX_BYTE:
        case I2C_STATE_TX_ADDR:
i2c_state_tx:
            // SCL ¯\_/¯
            // SDA ¯¯< b >

            if (phase == 0) {
                i2c_low(scl_bit);
                // Set or clear the SDA pin depending on the bit in the transmit byte
                uint32_t gpio = GPIO_BANK_01->dir;
                I2C_SDA_SET_OUTPUT_BIT(gpio);
                GPIO_BANK_01->dir = gpio;
                phase++;
                break;
            } else if (phase == 1) {
                i2c_hi(scl_bit);
                phase++;
                break;
            } else {
                I2C_HANDLE_CLOCK_STRETCH();
                phase = 0;
                if (work_bit == 0) {
                    if (state == I2C_STATE_TX_ADDR) {
                        state = I2C_STATE_R_ACK_ADDR;
                    } else {
                        state = I2C_STATE_R_ACK;
                    }
                } else {
                    work_bit--;
                    goto i2c_state_tx;
                }
                __attribute__((fallthrough));
            }

        case I2C_STATE_R_ACK:
        case I2C_STATE_R_ACK_ADDR:
            // SCL ¯\_/¯
            // SDA __< a >
            //         ^ sample SDA

            if (phase == 0) {
                i2c_low(scl_bit);
                i2c_hi(sda_bit);
                phase++;
                break;
            } else if (phase == 1) {
                i2c_hi(scl_bit);
                phase++;
                break;
            } else {
                I2C_HANDLE_CLOCK_STRETCH();

                I2C_CHECK_NAK();

                phase = 0;
                if (i2c->wlen == 0) {
                    if (state == I2C_STATE_R_ACK_ADDR && i2c->rlen > 0) {
                        // There is data to be a read, and we just sent the device address.
                        // We now turn around the bus and get ready for reading.
                        state = I2C_STATE_RX_BYTE;
                        work_bit = 7;
                        work_byte = 0;
                        goto i2c_state_rx;
                    } else if (i2c->rlen > 0) {
                        if (i2c->flags & PBDRV_RPROC_EV3_PRU1_I2C_CMD_NXT_QUIRK) {
                            // The ultrasonic sensor needs a Stop, an extra clock, and then a Start
                            state = I2C_STATE_STOP;
                            goto i2c_state_stop;
                        } else {
                            state = I2C_STATE_REP_START;
                            goto i2c_state_rep_start;
                        }
                    } else {
                        // End of transfer (including if it's an empty ping)
                        state = I2C_STATE_STOP;
                        goto i2c_state_stop;
                    }
                } else {
                    // There is data to send, either for a "write" type transaction
                    // or for the initial register number for a "read" type transaction
                    // before a repeated start (it works the same either way).
                    i2c->wlen--;
                    state = I2C_STATE_TX_BYTE;
                    work_bit = 7;
                    work_byte = *(i2c->buffer++);
                    goto i2c_state_tx;
                }
            }

        case I2C_STATE_STOP:
i2c_state_stop:
            // SCL ¯\__/¯¯
            // SDA ¯¯\__/¯

            if (phase == 0) {
                i2c_low(scl_bit);
                i2c_low(sda_bit);
                phase++;
            } else if (phase == 1) {
                i2c_hi(scl_bit);
                phase++;
            } else {
                I2C_HANDLE_CLOCK_STRETCH();
                i2c_hi(sda_bit);
                if (i2c->rlen) {
                    // Here we are generating the additional pulse needed for
                    // the NXT ultrasonic sensor. The bus signaling needed for
                    // this quirk happens to align completely with the way
                    // we have implemented a repeated start.
                    state = I2C_STATE_REP_START;
                    phase = 0;
                } else {
                    state = I2C_STATE_IDLE;
                    cmd->flags = PBDRV_RPROC_EV3_PRU1_I2C_PACK_FLAGS(
                        i2c->addr,
                        0,
                        0,
                        PBDRV_RPROC_EV3_PRU1_I2C_STAT_DONE | PBDRV_RPROC_EV3_PRU1_I2C_STAT_OK
                    );
                }
            }
            break;

        case I2C_STATE_REP_START:
i2c_state_rep_start:
            // SCL ¯\_/¯
            // SDA ../¯¯

            // Note that this state doesn't actually generate the
            // repeated start condition. It just generates a SCL pulse
            // which causes the device to release SDA. The actual
            // repeated start only happens in I2C_STATE_START.
            // This is why this state can be re-used for the NXT quirk:
            // SCL ¯\_/¯
            // SDA ¯¯¯¯¯

            if (phase == 0) {
                i2c_low(scl_bit);
                phase = 1;
            } else {
                i2c_hi(scl_bit);
                state = I2C_STATE_START;
                phase = 0;
            }
            break;

        case I2C_STATE_RX_BYTE:
i2c_state_rx:
            // SCL ¯\_/¯
            // SDA ..< b >
            //         ^ sample SDA

            if (phase == 0) {
                i2c_low(scl_bit);
                i2c_hi(sda_bit);
                phase++;
                break;
            } else if (phase == 1) {
                i2c_hi(scl_bit);
                phase++;
                break;
            } else {
                I2C_HANDLE_CLOCK_STRETCH();

                // Read the bit currently on SDA
                I2C_SDA_GET_INPUT_BIT();
                phase = 0;
                if (work_bit == 0) {
                    *(i2c->buffer++) = work_byte;
                    i2c->rlen--;
                    state = I2C_STATE_W_ACK;
                } else {
                    work_bit--;
                    goto i2c_state_rx;
                }
                __attribute__((fallthrough));
            }

        case I2C_STATE_W_ACK:
            // SCL ¯\_/¯
            // SDA ..< a >

            if (phase == 0) {
                i2c_low(scl_bit);
                if (i2c->rlen) {
                    i2c_low(sda_bit);
                } else {
                    i2c_hi(sda_bit);
                }
                phase++;
                break;
            } else if (phase == 1) {
                i2c_hi(scl_bit);
                phase++;
                break;
            } else {
                I2C_HANDLE_CLOCK_STRETCH();
                phase = 0;
                if (i2c->rlen) {
                    state = I2C_STATE_RX_BYTE;
                    work_bit = 7;
                    work_byte = 0;
                    goto i2c_state_rx;
                } else {
                    state = I2C_STATE_STOP;
                    goto i2c_state_stop;
                }
            }
    }

    // These code blocks do not normally execute but are instead
    // reached by asm goto statements in the above state machine.
    // They jump *into* the body of each of the if statements.

    if (0) {
handle_nak:
        cmd->flags = PBDRV_RPROC_EV3_PRU1_I2C_PACK_FLAGS(
            i2c->addr,
            i2c->rlen,
            i2c->wlen,
            PBDRV_RPROC_EV3_PRU1_I2C_STAT_DONE | PBDRV_RPROC_EV3_PRU1_I2C_STAT_NAK
        );
        state = I2C_STATE_IDLE;
    }

    if (0) {
handle_timeout:
        if (--i2c->timeout == 0) {
            i2c_hi(sda_bit);
            cmd->flags = PBDRV_RPROC_EV3_PRU1_I2C_PACK_FLAGS(
                i2c->addr,
                i2c->rlen,
                i2c->wlen,
                PBDRV_RPROC_EV3_PRU1_I2C_STAT_DONE | PBDRV_RPROC_EV3_PRU1_I2C_STAT_TIMEOUT
            );
            state = I2C_STATE_IDLE;
        }
    }

    // Flush updated state back to PRU memory
    i2c->state = state;
    i2c->phase = phase;
    i2c->work_byte = work_byte;
    i2c->work_bit = work_bit;
}

void main() {
    uint32_t pwms = 0;

    // We initialize this in code to avoid having to copy a data ram binary
    i2c_states[0].scl_bit = SENSOR_PORT_1_PIN_SCL;
    i2c_states[0].sda_bit = SENSOR_PORT_1_PIN_SDA;
    i2c_states[1].scl_bit = SENSOR_PORT_2_PIN_SCL;
    i2c_states[1].sda_bit = SENSOR_PORT_2_PIN_SDA;
    i2c_states[2].scl_bit = SENSOR_PORT_3_PIN_SCL;
    i2c_states[2].sda_bit = SENSOR_PORT_3_PIN_SDA;
    i2c_states[3].scl_bit = SENSOR_PORT_4_PIN_SCL;
    i2c_states[3].sda_bit = SENSOR_PORT_4_PIN_SDA;

    while (1) {
        // 24 MHz / 256 ==> 93.75 kHz tick rate for this counter
        uint8_t time_now = (*TIMER0_TIM34) >> 8;

        if (time_now == 0) {
            // 24 MHz / 256 / 256 ~= 366 Hz update rate
            pwms = SHARED_HACK->pwms;
        }

        update_pwm(pwms >> 0, time_now, LED0);
        update_pwm(pwms >> 8, time_now, LED1);
        // Intentional swap of 2 and 3
        // This puts the pins into the order R, G, R, G
        update_pwm(pwms >> 16, time_now, LED3);
        update_pwm(pwms >> 24, time_now, LED2);

        // Handle GPIO direction set/clear
        uint32_t val;
        if ((val = SHARED_HACK->gpio_bank_01_dir_set)) {
            GPIO_BANK_01->dir |= val;
            SHARED_HACK->gpio_bank_01_dir_set = 0;
        }
        if ((val = SHARED_HACK->gpio_bank_01_dir_clr)) {
            GPIO_BANK_01->dir &= ~val;
            SHARED_HACK->gpio_bank_01_dir_clr = 0;
        }

        // Handle I2C
        if ((*TIMER2_INTCTLSTAT) & TIMER_INTCTLSTAT_PRDINTSTAT12) {
            *TIMER2_INTCTLSTAT = TIMER_INTCTLSTAT_PRDINTSTAT12;

            // This code runs at 20 kHz, which is 2x the I2C clock rate
            handle_i2c(&SHARED->i2c[0], &i2c_states[0]);
            handle_i2c(&SHARED->i2c[1], &i2c_states[1]);
            handle_i2c(&SHARED->i2c[2], &i2c_states[2]);
            handle_i2c(&SHARED->i2c[3], &i2c_states[3]);
        }
    }
}

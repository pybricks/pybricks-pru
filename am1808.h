#ifndef __PRU_AM1808_H__
#define __PRU_AM1808_H__

// PRU0/1 Local INTC
#pragma ctable_entry 0  0x00004000
// Timer64P0
#pragma ctable_entry 1  0x01C20000
// I2C0
#pragma ctable_entry 2  0x01C22000
// PRU0/1 Local Data (self)
#pragma ctable_entry 3  0x00000000
// PRU1/0 Local Data (other)
#pragma ctable_entry 4  0x00002000
// MMC/SD
#pragma ctable_entry 5  0x01C40000
// SPI0
#pragma ctable_entry 6  0x01C41000
// UART0
#pragma ctable_entry 7  0x01C42000
// McASP0 DMA
#pragma ctable_entry 8  0x01D02000
// Reserved
#pragma ctable_entry 9  0x01D06000
// Reserved
#pragma ctable_entry 10 0x01D0A000
// UART1
#pragma ctable_entry 11 0x01D0C000
// UART2
#pragma ctable_entry 12 0x01D0D000
// USB0
#pragma ctable_entry 13 0x01E00000
// USB1
#pragma ctable_entry 14 0x01E25000
// UHPI Config
#pragma ctable_entry 15 0x01E10000
// Reserved
#pragma ctable_entry 16 0x01E12000
// I2C1
#pragma ctable_entry 17 0x01E28000
// EPWM0
#pragma ctable_entry 18 0x01F00000
// EPWM1
#pragma ctable_entry 19 0x01F02000
// Reserved
#pragma ctable_entry 20 0x01F04000
// ECAP0
#pragma ctable_entry 21 0x01F06000
// ECAP1
#pragma ctable_entry 22 0x01F07000
// ECAP2
#pragma ctable_entry 23 0x01F08000

/* 24 and 25 are variable */
// 24 = PRU0/1 Local Data   0x00000n00
// 25 = McASP0 Control      0x01D00n00

// Reserved
#pragma ctable_entry 26 0x01D04000
// Reserved
#pragma ctable_entry 27 0x01D08000

/* 28-31 are variable */
// 28 = DSP Megamodule RAM/ROM      0x11nnnn00
// 29 = EMIFA SDRAM                 0x40nnnn00
// 30 = Shared RAM                  0x80nnnn00
// 31 = mDDR/DDR2 Data              0xC0nnnn00

#endif

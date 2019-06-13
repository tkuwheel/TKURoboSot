/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'nios_processor' in SOPC Builder design 'sopc'
 * SOPC Builder design path: C:/Users/ICLab/Documents/GitHub/HuangRay/motion/DE0_NANO_6th/qsys/sopc.sopcinfo
 *
 * Generated: Thu Jun 13 10:08:25 CST 2019
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2_qsys"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x1000820
#define ALT_CPU_CPU_FREQ 100000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x00000000
#define ALT_CPU_CPU_IMPLEMENTATION "tiny"
#define ALT_CPU_DATA_ADDR_WIDTH 0x19
#define ALT_CPU_DCACHE_LINE_SIZE 0
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_DCACHE_SIZE 0
#define ALT_CPU_EXCEPTION_ADDR 0x800020
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 100000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 0
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 0
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 0
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_ICACHE_SIZE 0
#define ALT_CPU_INST_ADDR_WIDTH 0x19
#define ALT_CPU_NAME "nios_processor"
#define ALT_CPU_RESET_ADDR 0x800000


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x1000820
#define NIOS2_CPU_FREQ 100000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x00000000
#define NIOS2_CPU_IMPLEMENTATION "tiny"
#define NIOS2_DATA_ADDR_WIDTH 0x19
#define NIOS2_DCACHE_LINE_SIZE 0
#define NIOS2_DCACHE_LINE_SIZE_LOG2 0
#define NIOS2_DCACHE_SIZE 0
#define NIOS2_EXCEPTION_ADDR 0x800020
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 0
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 0
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 0
#define NIOS2_ICACHE_LINE_SIZE_LOG2 0
#define NIOS2_ICACHE_SIZE 0
#define NIOS2_INST_ADDR_WIDTH 0x19
#define NIOS2_RESET_ADDR 0x800000


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_NEW_SDRAM_CONTROLLER
#define __ALTERA_AVALON_PIO
#define __ALTERA_AVALON_TIMER
#define __ALTERA_AVALON_UART
#define __ALTERA_NIOS2_QSYS
#define __ALTPLL


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "Cyclone IV E"
#define ALT_ENHANCED_INTERRUPT_API_PRESENT
#define ALT_IRQ_BASE NULL
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/jtag_uart"
#define ALT_STDERR_BASE 0x10010c0
#define ALT_STDERR_DEV jtag_uart
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/jtag_uart"
#define ALT_STDIN_BASE 0x10010c0
#define ALT_STDIN_DEV jtag_uart
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/jtag_uart"
#define ALT_STDOUT_BASE 0x10010c0
#define ALT_STDOUT_DEV jtag_uart
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "sopc"


/*
 * altpll configuration
 *
 */

#define ALTPLL_BASE 0x10010b0
#define ALTPLL_IRQ -1
#define ALTPLL_IRQ_INTERRUPT_CONTROLLER_ID -1
#define ALTPLL_NAME "/dev/altpll"
#define ALTPLL_SPAN 16
#define ALTPLL_TYPE "altpll"
#define ALT_MODULE_CLASS_altpll altpll


/*
 * hal configuration
 *
 */

#define ALT_MAX_FD 32
#define ALT_SYS_CLK none
#define ALT_TIMESTAMP_CLK none


/*
 * jtag_uart configuration
 *
 */

#define ALT_MODULE_CLASS_jtag_uart altera_avalon_jtag_uart
#define JTAG_UART_BASE 0x10010c0
#define JTAG_UART_IRQ 0
#define JTAG_UART_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_NAME "/dev/jtag_uart"
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_SPAN 8
#define JTAG_UART_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8


/*
 * motor1 configuration
 *
 */

#define ALT_MODULE_CLASS_motor1 altera_avalon_pio
#define MOTOR1_BASE 0x10010a0
#define MOTOR1_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR1_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR1_CAPTURE 0
#define MOTOR1_DATA_WIDTH 16
#define MOTOR1_DO_TEST_BENCH_WIRING 0
#define MOTOR1_DRIVEN_SIM_VALUE 0
#define MOTOR1_EDGE_TYPE "NONE"
#define MOTOR1_FREQ 100000000
#define MOTOR1_HAS_IN 0
#define MOTOR1_HAS_OUT 1
#define MOTOR1_HAS_TRI 0
#define MOTOR1_IRQ -1
#define MOTOR1_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR1_IRQ_TYPE "NONE"
#define MOTOR1_NAME "/dev/motor1"
#define MOTOR1_RESET_VALUE 0
#define MOTOR1_SPAN 16
#define MOTOR1_TYPE "altera_avalon_pio"


/*
 * motor1_fb configuration
 *
 */

#define ALT_MODULE_CLASS_motor1_fb altera_avalon_pio
#define MOTOR1_FB_BASE 0x1001070
#define MOTOR1_FB_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR1_FB_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR1_FB_CAPTURE 0
#define MOTOR1_FB_DATA_WIDTH 32
#define MOTOR1_FB_DO_TEST_BENCH_WIRING 0
#define MOTOR1_FB_DRIVEN_SIM_VALUE 0
#define MOTOR1_FB_EDGE_TYPE "NONE"
#define MOTOR1_FB_FREQ 100000000
#define MOTOR1_FB_HAS_IN 1
#define MOTOR1_FB_HAS_OUT 0
#define MOTOR1_FB_HAS_TRI 0
#define MOTOR1_FB_IRQ -1
#define MOTOR1_FB_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR1_FB_IRQ_TYPE "NONE"
#define MOTOR1_FB_NAME "/dev/motor1_fb"
#define MOTOR1_FB_RESET_VALUE 0
#define MOTOR1_FB_SPAN 16
#define MOTOR1_FB_TYPE "altera_avalon_pio"


/*
 * motor2 configuration
 *
 */

#define ALT_MODULE_CLASS_motor2 altera_avalon_pio
#define MOTOR2_BASE 0x1001090
#define MOTOR2_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR2_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR2_CAPTURE 0
#define MOTOR2_DATA_WIDTH 16
#define MOTOR2_DO_TEST_BENCH_WIRING 0
#define MOTOR2_DRIVEN_SIM_VALUE 0
#define MOTOR2_EDGE_TYPE "NONE"
#define MOTOR2_FREQ 100000000
#define MOTOR2_HAS_IN 0
#define MOTOR2_HAS_OUT 1
#define MOTOR2_HAS_TRI 0
#define MOTOR2_IRQ -1
#define MOTOR2_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR2_IRQ_TYPE "NONE"
#define MOTOR2_NAME "/dev/motor2"
#define MOTOR2_RESET_VALUE 0
#define MOTOR2_SPAN 16
#define MOTOR2_TYPE "altera_avalon_pio"


/*
 * motor2_fb configuration
 *
 */

#define ALT_MODULE_CLASS_motor2_fb altera_avalon_pio
#define MOTOR2_FB_BASE 0x1001060
#define MOTOR2_FB_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR2_FB_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR2_FB_CAPTURE 0
#define MOTOR2_FB_DATA_WIDTH 32
#define MOTOR2_FB_DO_TEST_BENCH_WIRING 0
#define MOTOR2_FB_DRIVEN_SIM_VALUE 0
#define MOTOR2_FB_EDGE_TYPE "NONE"
#define MOTOR2_FB_FREQ 100000000
#define MOTOR2_FB_HAS_IN 1
#define MOTOR2_FB_HAS_OUT 0
#define MOTOR2_FB_HAS_TRI 0
#define MOTOR2_FB_IRQ -1
#define MOTOR2_FB_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR2_FB_IRQ_TYPE "NONE"
#define MOTOR2_FB_NAME "/dev/motor2_fb"
#define MOTOR2_FB_RESET_VALUE 0
#define MOTOR2_FB_SPAN 16
#define MOTOR2_FB_TYPE "altera_avalon_pio"


/*
 * motor3 configuration
 *
 */

#define ALT_MODULE_CLASS_motor3 altera_avalon_pio
#define MOTOR3_BASE 0x1001080
#define MOTOR3_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR3_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR3_CAPTURE 0
#define MOTOR3_DATA_WIDTH 16
#define MOTOR3_DO_TEST_BENCH_WIRING 0
#define MOTOR3_DRIVEN_SIM_VALUE 0
#define MOTOR3_EDGE_TYPE "NONE"
#define MOTOR3_FREQ 100000000
#define MOTOR3_HAS_IN 0
#define MOTOR3_HAS_OUT 1
#define MOTOR3_HAS_TRI 0
#define MOTOR3_IRQ -1
#define MOTOR3_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR3_IRQ_TYPE "NONE"
#define MOTOR3_NAME "/dev/motor3"
#define MOTOR3_RESET_VALUE 0
#define MOTOR3_SPAN 16
#define MOTOR3_TYPE "altera_avalon_pio"


/*
 * motor3_fb configuration
 *
 */

#define ALT_MODULE_CLASS_motor3_fb altera_avalon_pio
#define MOTOR3_FB_BASE 0x1001050
#define MOTOR3_FB_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR3_FB_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR3_FB_CAPTURE 0
#define MOTOR3_FB_DATA_WIDTH 32
#define MOTOR3_FB_DO_TEST_BENCH_WIRING 0
#define MOTOR3_FB_DRIVEN_SIM_VALUE 0
#define MOTOR3_FB_EDGE_TYPE "NONE"
#define MOTOR3_FB_FREQ 100000000
#define MOTOR3_FB_HAS_IN 1
#define MOTOR3_FB_HAS_OUT 0
#define MOTOR3_FB_HAS_TRI 0
#define MOTOR3_FB_IRQ -1
#define MOTOR3_FB_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR3_FB_IRQ_TYPE "NONE"
#define MOTOR3_FB_NAME "/dev/motor3_fb"
#define MOTOR3_FB_RESET_VALUE 0
#define MOTOR3_FB_SPAN 16
#define MOTOR3_FB_TYPE "altera_avalon_pio"


/*
 * sdram configuration
 *
 */

#define ALT_MODULE_CLASS_sdram altera_avalon_new_sdram_controller
#define SDRAM_BASE 0x800000
#define SDRAM_CAS_LATENCY 3
#define SDRAM_CONTENTS_INFO
#define SDRAM_INIT_NOP_DELAY 0.0
#define SDRAM_INIT_REFRESH_COMMANDS 2
#define SDRAM_IRQ -1
#define SDRAM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SDRAM_IS_INITIALIZED 1
#define SDRAM_NAME "/dev/sdram"
#define SDRAM_POWERUP_DELAY 200.0
#define SDRAM_REFRESH_PERIOD 7.8125
#define SDRAM_REGISTER_DATA_IN 1
#define SDRAM_SDRAM_ADDR_WIDTH 0x16
#define SDRAM_SDRAM_BANK_WIDTH 2
#define SDRAM_SDRAM_COL_WIDTH 8
#define SDRAM_SDRAM_DATA_WIDTH 16
#define SDRAM_SDRAM_NUM_BANKS 4
#define SDRAM_SDRAM_NUM_CHIPSELECTS 1
#define SDRAM_SDRAM_ROW_WIDTH 12
#define SDRAM_SHARED_DATA 0
#define SDRAM_SIM_MODEL_BASE 0
#define SDRAM_SPAN 8388608
#define SDRAM_STARVATION_INDICATOR 0
#define SDRAM_TRISTATE_BRIDGE_SLAVE ""
#define SDRAM_TYPE "altera_avalon_new_sdram_controller"
#define SDRAM_T_AC 5.5
#define SDRAM_T_MRD 3
#define SDRAM_T_RCD 20.0
#define SDRAM_T_RFC 70.0
#define SDRAM_T_RP 20.0
#define SDRAM_T_WR 14.0


/*
 * sys_timer configuration
 *
 */

#define ALT_MODULE_CLASS_sys_timer altera_avalon_timer
#define SYS_TIMER_ALWAYS_RUN 0
#define SYS_TIMER_BASE 0x1001000
#define SYS_TIMER_COUNTER_SIZE 32
#define SYS_TIMER_FIXED_PERIOD 0
#define SYS_TIMER_FREQ 100000000
#define SYS_TIMER_IRQ 2
#define SYS_TIMER_IRQ_INTERRUPT_CONTROLLER_ID 0
#define SYS_TIMER_LOAD_VALUE 99
#define SYS_TIMER_MULT 1.0E-6
#define SYS_TIMER_NAME "/dev/sys_timer"
#define SYS_TIMER_PERIOD 1
#define SYS_TIMER_PERIOD_UNITS "us"
#define SYS_TIMER_RESET_OUTPUT 0
#define SYS_TIMER_SNAPSHOT 1
#define SYS_TIMER_SPAN 32
#define SYS_TIMER_TICKS_PER_SEC 1000000.0
#define SYS_TIMER_TIMEOUT_PULSE_OUTPUT 0
#define SYS_TIMER_TYPE "altera_avalon_timer"


/*
 * tx configuration
 *
 */

#define ALT_MODULE_CLASS_tx altera_avalon_pio
#define TX_BASE 0x1001040
#define TX_BIT_CLEARING_EDGE_REGISTER 0
#define TX_BIT_MODIFYING_OUTPUT_REGISTER 0
#define TX_CAPTURE 1
#define TX_DATA_WIDTH 1
#define TX_DO_TEST_BENCH_WIRING 0
#define TX_DRIVEN_SIM_VALUE 0
#define TX_EDGE_TYPE "RISING"
#define TX_FREQ 100000000
#define TX_HAS_IN 1
#define TX_HAS_OUT 0
#define TX_HAS_TRI 0
#define TX_IRQ 3
#define TX_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TX_IRQ_TYPE "EDGE"
#define TX_NAME "/dev/tx"
#define TX_RESET_VALUE 0
#define TX_SPAN 16
#define TX_TYPE "altera_avalon_pio"


/*
 * uart_rs232 configuration
 *
 */

#define ALT_MODULE_CLASS_uart_rs232 altera_avalon_uart
#define UART_RS232_BASE 0x1001020
#define UART_RS232_BAUD 115200
#define UART_RS232_DATA_BITS 8
#define UART_RS232_FIXED_BAUD 1
#define UART_RS232_FREQ 100000000
#define UART_RS232_IRQ 1
#define UART_RS232_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART_RS232_NAME "/dev/uart_rs232"
#define UART_RS232_PARITY 'N'
#define UART_RS232_SIM_CHAR_STREAM ""
#define UART_RS232_SIM_TRUE_BAUD 0
#define UART_RS232_SPAN 32
#define UART_RS232_STOP_BITS 1
#define UART_RS232_SYNC_REG_DEPTH 2
#define UART_RS232_TYPE "altera_avalon_uart"
#define UART_RS232_USE_CTS_RTS 0
#define UART_RS232_USE_EOP_REGISTER 0

#endif /* __SYSTEM_H_ */

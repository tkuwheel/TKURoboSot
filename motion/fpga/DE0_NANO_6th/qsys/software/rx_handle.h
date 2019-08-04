/*
 * rx_handle.h
 *
 *  Created on: 2019/6/13
 *      Author: ICLab
 */

#ifndef RX_HANDLE_H_

#include <stdio.h>
#include <system.h>
#include <unistd.h>

#include "sys/alt_irq.h"
#include "alt_types.h"

#include "altera_avalon_pio_regs.h"
#include "Crc16.h"
#include "parameter.h"
//#define _DEBUG
#define RX_HANDLE_H_

#ifdef UART_RS232_BASE

    void handle_rx_interrupts(void* context);

    //init_urat_irq
    void init_rx_irq(bool &receive );

    void get(bool &receive);
#endif /* UART_RS232_BASE */

#endif /* RX_HANDLE_H_ */

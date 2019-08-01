/*
 * tx_handle.h
 *
 *  Created on: 2019/6/13
 *      Author: ICLab
 */

#ifndef TX_HANDLE_H_

#include <stdio.h>
#include <system.h>
#include <unistd.h>

#include "sys/alt_irq.h"
#include "alt_types.h"

#include "altera_avalon_pio_regs.h"

#include "Crc16.h"
#include "parameter.h"
#define TX_HANDLE_H_
#ifdef TX_BASE
    void handle_tx_interrupts(void* context);


    void init_tx_irq(bool &transmit);	//init_urat_irq

    void send(bool &transmit);

#endif /* TX_BASE */


#endif /* TX_HANDLE_H_ */

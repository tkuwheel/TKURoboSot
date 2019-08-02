/*
 * tx_handle.cpp
 *
 *  Created on: 2019/7/18
 *      Author: ICLab
 */
/*
 * tx_handle.h
 *
 *  Created on: 2019/6/13
 *      Author: ICLab
 */
#include <stdio.h>
#include <system.h>
#include <unistd.h>

#include "sys/alt_irq.h"
#include "alt_types.h"

#include "altera_avalon_pio_regs.h"

#include "Crc16.h"
#include "parameter.h"

alt_u8 tx_data[16]={HEAD_1, HEAD_2};

int c = 0;
int tx_count = 0;
void handle_tx_interrupts(void* context)
{
	/* Cast context to edge_capture's type. It is important that this be
	 * declared volatile to avoid unwanted compiler optimization.
	 */
	bool* transmit_ptr = (bool*) context;
	*transmit_ptr = true;
	//	*edge_capture_ptr = c++;
	//	printf("trigger inturrupt\n");
	/* Store the value in the Button's edge capture register in *context. */
	//	printf("edge_capture_value=0x%x\n",*edge_capture_ptr);
	//	printf("inturruptmask_value=0x%x\n",IORD(PIO_0_BASE,2));
	/* Reset the Button's edge capture register. */
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(TX_BASE, 0);
	// count direction


}
//init_urat_irq
void init_tx_irq(bool &transmit)
{
	/* Recast the edge_capture pointer to match the alt_irq_register() function
	 * prototype. */
	//	void* edge_capture_ptr  = (void*) &edge_capture;
	void* transmit_ptr = (void*) &transmit;
	/* Enable all 4 button interrupts. */
	IOWR(TX_BASE, 2,0x1);
	/* Reset the edge capture register. */
	IOWR(TX_BASE, 3,0x0);
	/* Register the interrupt handler. */

	alt_ic_isr_register(
		TX_IRQ_INTERRUPT_CONTROLLER_ID,
		TX_IRQ,
		handle_tx_interrupts,
		transmit_ptr,
		0
		);
	printf("tx inturrupt enable\n");
	//	alt_ic_irq_enable(UART_IRQ_INTERRUPT_CONTROLLER_ID, UART_IRQ);

}
void send(bool &transmit)
{
	Crc_16 Crc;
	IOWR_ALTERA_AVALON_PIO_DATA(LED_BASE,1);
	int j = 0;
	int motor1_fb = IORD_ALTERA_AVALON_PIO_DATA(MOTOR1_FB_BASE);
	int motor2_fb = IORD_ALTERA_AVALON_PIO_DATA(MOTOR2_FB_BASE);
	int motor3_fb = IORD_ALTERA_AVALON_PIO_DATA(MOTOR3_FB_BASE);
	tx_data[2] = (motor1_fb >> 24) & 0xff;
	tx_data[3] = (motor1_fb >> 16) & 0xff;
	tx_data[4] = (motor1_fb >> 8) & 0xff;
	tx_data[5] = (motor1_fb) & 0xff;
	tx_data[6] = (motor2_fb >> 24) & 0xff;
	tx_data[7] = (motor2_fb >> 16) & 0xff;
	tx_data[8] = (motor2_fb >> 8) & 0xff;
	tx_data[9] = (motor2_fb) & 0xff;
	tx_data[10] = (motor3_fb >> 24) & 0xff;
	tx_data[11] = (motor3_fb >> 16) & 0xff;
	tx_data[12] = (motor3_fb >> 8) & 0xff;
	tx_data[13] = (motor3_fb) & 0xff;
	alt_u16 crc_16 = Crc.getCrc(tx_data, 14);
	tx_data[14] = crc_16 >> 8 & 0xff;
	tx_data[15] = crc_16 & 0xff;
	while(transmit){
		if(IORD(UART_RS232_BASE,2) & 0x40){
			if( IORD(UART_RS232_BASE,2) & 0x0100){
				IOWR(UART_RS232_BASE,2,0);
			}else{
				IOWR(UART_RS232_BASE,1,tx_data[j++]);
				if(j >= 16){
					j=0;
					transmit = false;
					IOWR(UART_RS232_BASE,2,0);
	//					led = ~led;
					break;

				}

			}
		}
	}
	IOWR_ALTERA_AVALON_PIO_DATA(LED_BASE,0);
//		printf("transmit finished\n");
}






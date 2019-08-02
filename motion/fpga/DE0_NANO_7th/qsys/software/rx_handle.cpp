/*
 * rx_handle.cpp
 *
 *  Created on: 2019/7/18
 *      Author: ICLab
 */

#include "rx_handle.h"

alt_u8 rx_data[15];
int i;
int error_count = 0;
void handle_rx_interrupts(void* context)
{
	/* Cast context to edge_capture's type. It is important that this be
	 * declared volatile to avoid unwanted compiler optimization.
	 */
	bool* receive_ptr = (bool*) context;
	*receive_ptr = false;
	/* Store the value in the Button's edge capture register in *context. */

//        printf("status=0x%x,\t", IORD(UART_BASE,3));
//        printf("context=0x%x\n",*data);
	/* Reset the Button's edge capture register. */
//        IOWR(UART_BASE,3,0x00);
	alt_u8 data;
	if(IORD(UART_RS232_BASE,2))
	if(IORD(UART_RS232_BASE,2) & 0x080){

		if( IORD(UART_RS232_BASE,2) & 0x0100)
		{
//		    printf("error rxdata ..!\n");
			IOWR(UART_RS232_BASE,2,0x00);
		} else {
			data = IORD(UART_RS232_BASE,0);
			if((i==0)&&(data==HEAD_1)){
				rx_data[i] = data;
				i++;
			}else if((i==1)&&(data==HEAD_2)){
				rx_data[i] = data;
				i++;
			}else if(i>=2){
				rx_data[i] = data;
				i++;
			}else{
				i=0;
			}




			if(i>=15){
				i=0;
				*receive_ptr = true;
			}
//		    printf("Your character rxd is:\t%x  %d\n", rxdata,count++);
			IOWR(UART_RS232_BASE,2,0x00);
		}
	}
	// count direction

}
//init_urat_irq
void init_rx_irq(bool &receive )
{
	/* Recast the edge_capture pointer to match the alt_irq_register() function
	 * prototype. */
	void* receive_ptr  = (void*) &receive;
	/* Enable all 4 button interrupts. */
//        IOWR(BUTTON_PIO_BASE, 2,0xf);
	/* Reset the edge capture register. */
//        IOWR(BUTTON_PIO_BASE, 3,0x0);
	/* Register the interrupt handler. */
	IOWR(UART_RS232_BASE, 3,0x80);

	alt_ic_isr_register(
		UART_RS232_IRQ_INTERRUPT_CONTROLLER_ID,
		UART_RS232_IRQ,
		handle_rx_interrupts,
		receive_ptr,
		0
		);
	printf("rx inturrupt enable\n");
//        alt_ic_irq_enable(UART_IRQ_INTERRUPT_CONTROLLER_ID, UART_IRQ);

}
void get(bool &receive)
{
	Crc_16 Crc;
	alt_u8 crc_data[15];
	alt_u8 data[15];
	alt_16 motor1, motor2, motor3;
	alt_u8 signal = 0;
	alt_u8 shoot = 0;
	alt_u8 holdBall_L;
	alt_u8 holdBall_R;
	alt_u8 holdBall_DIR;
	std::copy(rx_data, rx_data+15, data);
//    	for(int head=0; head<12; head++){
	if((data[0] == HEAD_1) && (data[1] == HEAD_2)){
//		for(int sort = 0; sort < 15; sort++){
//			crc_data[sort] = data[sort];
//		}
		alt_u16 crc_16 = Crc.getCrc(data, 15);
		if(crc_16 == 0){
			IOWR_ALTERA_AVALON_PIO_DATA(RX_BASE,1);
			IOWR_ALTERA_AVALON_PIO_DATA(LED_BASE,2);
			motor1 = ((data[2] << 8) + data[3]) & 0xffff;
			motor2 = ((data[4] << 8) + data[5]) & 0xffff;
			motor3 = ((data[6] << 8) + data[7]) & 0xffff;
			signal = data[8] & 0xff;
			shoot = data[9] & 0xff;
			holdBall_L = data[10] & 0xff;
			holdBall_R = data[11] & 0xff;
			holdBall_DIR = data[12] & 0xff;
			IOWR_ALTERA_AVALON_PIO_DATA(MOTOR1_BASE, motor1);
			IOWR_ALTERA_AVALON_PIO_DATA(MOTOR2_BASE, motor2);
			IOWR_ALTERA_AVALON_PIO_DATA(MOTOR3_BASE, motor3);
			IOWR_ALTERA_AVALON_PIO_DATA(MOTORSIG_BASE, signal);
			IOWR_ALTERA_AVALON_PIO_DATA(SHOOT_BASE, shoot);
			IOWR_ALTERA_AVALON_PIO_DATA(HOLD_BALL_L_BASE, holdBall_L);
			IOWR_ALTERA_AVALON_PIO_DATA(HOLD_BALL_R_BASE, holdBall_R);
			IOWR_ALTERA_AVALON_PIO_DATA(HOLD_BALL_DIR_BASE, holdBall_DIR);
//			IOWR_ALTERA_AVALON_PIO_DATA(RX_BASE,0);
			IOWR_ALTERA_AVALON_PIO_DATA(LED_BASE,0);
#ifdef _DEBUG
			printf("motor1 %d motor2 %d motor3 %d en&stop %x shoot %x\n ", motor1, motor2, motor3, data[8], data[9]);
			printf("pwm1 %d pwm2 %d dir %x\n ", holdBall_L, holdBall_R, holdBall_DIR);
#endif
		}else{
			error_count++;
#ifdef _DEBUG
			printf("get %d wrong packet\n ", error_count);
			for(int sort = 0; sort < 15; sort++){
				printf("%x ", data[sort]);
			}
			printf("\n");
#endif
		}

	}


	receive = false;

}

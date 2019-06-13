/*
 * "Hello World" example.
 *
 * This example prints 'Hello from Nios II' to the STDOUT stream. It runs on
 * the Nios II 'standard', 'full_featured', 'fast', and 'low_cost' example
 * designs. It runs with or without the MicroC/OS-II RTOS and requires a STDOUT
 * device in your system's hardware.
 * The memory footprint of this hosted application is ~69 kbytes by default
 * using the standard reference design.
 *
 * For a reduced footprint version of this template, and an explanation of how
 * to reduce the memory footprint for a given application, see the
 * "small_hello_world" template.
 *
 */

#include <stdio.h>
#include <system.h>
#include <unistd.h>

#include "sys/alt_irq.h"
#include "alt_types.h"

#include "altera_avalon_pio_regs.h"

#include "tx_handle.h"
#include "rx_handle.h"

/* A "loop counter" variable. */
int count;
/* A variable to hold the value of the button pio edge capture register. */




int main()
{
    printf("Hello fira!\n");
//    int i;
	bool transmit = false;
	bool receive = false;
    int j = 0;
    /* Initialize the button pio. */


    init_rx_irq(receive);
    init_tx_irq(transmit);

    while( 1 )
    {

//    	usleep(1000);
//    	printf("transmit: %d\n", transmit);
    	if(transmit == true)send(transmit);
    	if(receive == true)get(receive);
//    	printf("get rx data: ");
//    	for(int counter = 0; counter<12; counter++){
//        	printf("%x ",rx_data[counter]);
//    	}
//    	printf("\n");
    }

  return 0;
}


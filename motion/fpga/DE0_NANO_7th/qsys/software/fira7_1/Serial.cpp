/*
 * Serial.cpp
 *
 *  Created on: 2019/7/18
 *      Author: ICLab
 */

#include <uinstd.h>
#include "Serial.h"

Serial::Serial() {
	// TODO Auto-generated constructor stub
	m_receive = false;
	m_transmit = false;
}

Serial::~Serial() {
	// TODO Auto-generated destructor stub
}

void Serial::Run() {
    init_rx_irq(m_receive);
    init_tx_irq(m_transmit);

    while( 1 )
    {

    	if(m_transmit == true)send(m_transmit);
    	if(m_receive == true)get(m_receive);
    	usleep(1000);
    }
}

/*
 * Serial.h
 *
 *  Created on: 2019/7/18
 *      Author: ICLab
 */

#ifndef SERIAL_H_
#define SERIAL_H_
#include <unistd.h>
#include "rx_handle.h"
#include "tx_handle.h"
class Serial {
public:
	Serial();
	virtual ~Serial();

	void Run();
private:
	bool m_receive;
	bool m_transmit;
};

#endif /* SERIAL_H_ */

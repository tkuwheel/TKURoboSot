parameter FEEDBACK_FREQUENCY = 10;     // Hz
parameter RX_MOTOR_SIZE = 2;       //bytes
parameter TX_MOTOR_SIZE = 4;       //bytes
parameter RX_PACKAGE_SIZE = 12;     //bytes RX size (ff fa w1 w2 w3 en+stop shoot crc1 crc2)
parameter TX_PACKAGE_SIZE = 16;     //bytes TX size (Feedback: ff fa w1 w2 w3 crc1 crc2)

parameter FEEDBACK_FREQUENCY = 100;  // Hz

parameter RX_MOTOR_SIZE = 2;        //bytes
parameter TX_MOTOR_SIZE = 4;        //bytes
parameter RX_PACKAGE_SIZE = 12;     //bytes RX size (ff fa w1 w2 w3 0 shoot crc1 crc2)
parameter TX_PACKAGE_SIZE = 16;     //bytes TX size (Feedback: ff fa w1 w2 w3 crc1 crc2)
parameter MAX_VELOCITY = 6930;      //rpm
parameter MIN_VELOCITY = 15;        //rpm
parameter MIN_DUTY = 51;
parameter MAX_DUTY = 460;
parameter Threshold_Diff_Vel = 15;

parameter P_GAIN = 100;
parameter I_GAIN = 300;
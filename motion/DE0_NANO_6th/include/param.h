parameter FEEDBACK_FREQUENCY = 100;  // Hz
parameter FEEDBACK_PERIOD = 1000* 1/FEEDBACK_FREQUENCY;  // ms
parameter RX_MOTOR_SIZE = 2;        //bytes
parameter TX_MOTOR_SIZE = 4;        //bytes
parameter RX_PACKAGE_SIZE = 12;     //bytes RX size (ff fa w1 w2 w3 en+stop shoot crc1 crc2)
parameter TX_PACKAGE_SIZE = 16;     //bytes TX size (Feedback: ff fa w1 w2 w3 crc1 crc2)
parameter MAX_VELOCITY = 7580;      //rpm
parameter MIN_VELOCITY = 15;        //rpm
parameter MIN_DUTY = 16'h3E;
parameter MAX_DUTY = 16'h260;
parameter Threshold_Diff_Vel = 32;

parameter P_GAIN = 3;
parameter I_GAIN = 10;
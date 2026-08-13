### STM32 UART Practice
I learned how to control a motor pump with an STM32 Nucleo G070RB board. Go to Core/SRC/main.c to see the most up to date firmware. Currently, I use a modbus 0x06 command to do PID control with different register addresses. There are two UART lines: one for displaying the RPM of the motor and one to transmit and receive what I've typed into the console. 

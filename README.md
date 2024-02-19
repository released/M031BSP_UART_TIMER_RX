# M031BSP_UART_TIMER_RX
 M031BSP_UART_TIMER_RX


udpate @ 2024/02/19

1. use UART2 (PC.5,PC.4) + TIMER3 as uart idle line timer 

- time out set to 217 us (2.5 bytes timing for bus idle detection) @ BAUD RATE 115200

- BAUD RATE 115200 = 86.8 us / per Byte 

- UART : start bit + Bytes + stop bit = 10 bit , transmit 1 Byte(10 bit) need 1/115200(baud rate)*10(bit) = 86.8us

- uart buffer set to 4096 bytes

2. use text_1218bytes to test transfer data

3. below capture : use another UART bridge to send data

![image](https://github.com/released/M031BSP_UART_TIMER_RX/blob/main/uart_tx.jpg)

4. below capture : MCU print log (UART0) when receive data (UART2 RX) from another UART bridge

![image](https://github.com/released/M031BSP_UART_TIMER_RX/blob/main/mcu_rx.jpg)



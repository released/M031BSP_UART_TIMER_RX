/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_REVERSE1                   			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)

struct flag_32bit flag_RS485_CTL;
#define FLAG_RS485_RCV_DATA_FINISH                 		(flag_RS485_CTL.bit0)
#define FLAG_RS485_TXPACKET_START                   	(flag_RS485_CTL.bit1)
#define FLAG_RS485_TXPACKET_SEND                   	    (flag_RS485_CTL.bit2)

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


#define RS485_BUFFER_SIZE                       (1024*4)//(256)
#define RS485_PORT                              (UART2)
#define nRTSPin                                 (PC3)
#define RECEIVE_MODE                            (0)
#define TRANSMIT_MODE                           (1)

volatile uint16_t usRcvBufferLastByte = 0;
volatile uint16_t usRcvBufferPos = 0;
volatile uint8_t rs485_rx_buffer[RS485_BUFFER_SIZE] = {0}; // UART Rx received data Buffer (RAM)
uint8_t rs485_tx_packet[RS485_BUFFER_SIZE] = {0};
uint8_t rs485_packet_counter = 0;
uint8_t rs485_packet_len = 0;


typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eUARTRcvState;

volatile eUARTRcvState eRcvState = STATE_RX_ERROR;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void RS485_GetData(UART_T *uart , uint8_t* pucByte);

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(uint16_t ms)
{
	#if 1
    uint32_t tickstart = get_tick();
    uint32_t wait = ms;
	uint32_t tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}


void vRS485PortTimersEnable(void)
{
    // TIMER_ResetCounter(TIMER3);
	TIMER3->CTL |= TIMER_CTL_RSTCNT_Msk;
    TIMER_Start(TIMER3);
}

void vRS485PortTimersDisable(void)
{
    TIMER_Stop(TIMER3);
    // TIMER_ResetCounter(TIMER3);
	TIMER3->CTL |= TIMER_CTL_RSTCNT_Msk;
}


void vRS485PortClearRXFIFO(void)
{
	while (UART_RX_IDLE(RS485_PORT) == 0);
	RS485_PORT->FIFO |= UART_FIFO_RXRST_Msk;
	while(RS485_PORT->FIFO & UART_FIFO_RXRST_Msk);	
}

void vRS485PortResetState(void)
{
	usRcvBufferPos = 0;					
	eRcvState = STATE_RX_IDLE;
}

void prvvTIMERExpiredISR(void)
{
	usRcvBufferLastByte = usRcvBufferPos - 1;

    /*
        buffer condition check
    */

	if (eRcvState == STATE_RX_RCV)
	{
		UART_DisableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);

		FLAG_RS485_RCV_DATA_FINISH = 1;	
	}

	vRS485PortTimersDisable();   
	eRcvState = STATE_RX_IDLE;
}

void prvvUARTRxISR(void)
{
    uint8_t ucByte;
    RS485_GetData(RS485_PORT , &ucByte);

    switch ( eRcvState )
    {
        case STATE_RX_INIT:
            vRS485PortTimersEnable();
            break;

        case STATE_RX_ERROR:
            vRS485PortTimersEnable();
            break;

        case STATE_RX_IDLE:	
            usRcvBufferPos = 0;
            rs485_rx_buffer[usRcvBufferPos++] = ucByte;
			// PF2 = !PF2;	//DEBUG
			eRcvState = STATE_RX_RCV;
			/* Enable t3.5 timers. */
			vRS485PortTimersEnable();
            break;

        case STATE_RX_RCV:
            if( usRcvBufferPos < RS485_BUFFER_SIZE )
            {					
                rs485_rx_buffer[usRcvBufferPos++] = ucByte;
				// PF2 = !PF2;	//DEBUG
            }
            else
            {
				usRcvBufferPos = 0;	
                eRcvState = STATE_RX_ERROR;
				// vRS485PortClearRXFIFO();
				vRS485PortResetState();			
            }
            vRS485PortTimersEnable();
            break;
    }
}

void prvvUARTTxISR(void)
{
	// uint16_t i = 0;

	//THERINT Flag Cleared By Write UART_DAT
	if (FLAG_RS485_TXPACKET_START)                 //UART buffer in use
	{
		#if 0	//decrease
		while (rs485_packet_counter)
		{
			UART_WRITE(RS485_PORT, rs485_tx_packet[i++]);
			while(UART_IS_TX_FULL(RS485_PORT));
			rs485_packet_counter--;
		}

		if (rs485_packet_counter == 0)
		{
			
			UART_DisableInt(RS485_PORT, UART_INTEN_THREIEN_Msk);
			rs485_packet_counter = 0;                //initial
			FLAG_RS485_TXPACKET_START = 0;            		//initial
		}

		#else
		if (rs485_packet_counter == rs485_packet_len )
		{			
			/* No more data, just stop Tx (Stop work) */
			UART_DisableInt(RS485_PORT, UART_INTEN_THREIEN_Msk);
			rs485_packet_counter = 0;                //initial
			FLAG_RS485_TXPACKET_START = 0;            		//initial                
			// nRTSPin = RECEIVE_MODE;	                
			// UART_EnableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);
		}
		else
		{
			UART_WRITE(RS485_PORT, rs485_tx_packet[rs485_packet_counter]);
			while(UART_IS_TX_FULL(RS485_PORT));
			rs485_packet_counter++;
			// printf("%2d\r\n",rs485_packet_counter);
		}
		#endif
	}
}

void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

    	prvvTIMERExpiredISR();
    }
}



void RS485_StartTxData(uint8_t* Buffer_Input, uint32_t Buffer_len)
{	
    // nRTSPin = TRANSMIT_MODE;
	// __disable_irq();
	// UART_DisableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);
	// NVIC_DisableIRQ(UART02_IRQn);  
	nRTSPin = TRANSMIT_MODE;

	/* 
		TODO: make sure receive pin change to low after last byte transmit
		last byte wille be ignore due to RST pin alreay become low
	*/

	#if 0	//polling
    for (i = 0; i < Buffer_len + 1; i++)	
    {
        while(UART_GET_TX_FULL(RS485_PORT));
        UART_WRITE(RS485_PORT, Buffer_Input[i]);
    }

	#else	// interrupt
	FLAG_RS485_TXPACKET_START = 1;
	// rs485_packet_counter = Buffer_len;	//decrease
	rs485_packet_counter = 0;
	UART_EnableInt(RS485_PORT, UART_INTEN_THREIEN_Msk);	
	while(FLAG_RS485_TXPACKET_START == 1);
	#endif
	
    UART_WAIT_TX_EMPTY(RS485_PORT);
	vRS485PortClearRXFIFO();

	nRTSPin = RECEIVE_MODE;	 

	// NVIC_EnableIRQ(UART02_IRQn);	
	// __enable_irq();
	// UART_EnableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);	


}


void RS485_GetData(UART_T *uart , uint8_t* pucByte)
{

    // while((uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0);
    *pucByte = uart->DAT;

	// while(UART_GET_RX_EMPTY(uart) != 0);
	// *pucByte = UART_READ(uart);
}


/*
	baudrate 750000 : 13.3 us / Byte
					1.5 Bytes = 19.9 us 
					2 Bytes = 26.6 us  
					2.5 Bytes = 33.25 us
					3 Bytes = 39.9 us
					3.5 Bytes = 46.55 us
	baudrate 115200 : 86.8 us / Byte
					1.5 Bytes = 130.2 us
					2 Bytes = 173.6 us 
					2.5 Bytes = 217 us
					3 Bytes = 260.4 us
					3.5 Bytes = 303.8 us

	target timeout RX idle: 
	1000us/1000 Hz
	500 us/2000 Hz
	250 us/4000 Hz
	200 us/5000 Hz
	190 us/5263 Hz
	180 us/5556 Hz
	170 us/5882 Hz
	160 us/6250 Hz
	150 us/6666 Hz
	125 us/8000 Hz
	100 us/10000 Hz
	50 us/20000 Hz
	40 us/25000 Hz
	30 us/33333 Hz
	20 us/50000 Hz

*/

void RS485_SetTimeout(uint16_t us)    // delay
{

	uint32_t result = 0;
    /* Reset IP TMR0 */
    SYS_ResetModule(TMR3_RST);
    TIMER_DisableInt(TIMER3);

	// vRS485PortTimersDisable();

   	#if 1
	result = 1000000/us;
    TIMER_Open(TIMER3, TIMER_ONESHOT_MODE, result );   
	#else
    uint16_t Hz = 0;

    switch(us)
    {
        case 1000:
            Hz = 1000;
            break;
        case 500:
            Hz = 2000;
            break;
        case 250:
            Hz = 4000;
            break;
        case 200:
            Hz = 5000;
            break;
        case 190:
            Hz = 5263;
            break;
        case 180:
            Hz = 5556;
            break;
        case 170:
            Hz = 5882;
            break;
        case 160:
            Hz = 6250;
            break;
        case 150:  
            Hz = 6666;
			break;
        case 125:  
            Hz = 8000;
			break;  
        case 100:  
            Hz = 10000;
			break;  
        case 50:  
            Hz = 20000;
			break;   
        case 40:  
            Hz = 25000;
			break;
        case 30:  
            Hz = 33333;
			break;          
        default :
            Hz = 6666;
            break;
    }
	
    TIMER_Open(TIMER3, TIMER_ONESHOT_MODE, Hz );   

	#endif


    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER3);
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR3_IRQn);
    NVIC_SetPriority(TMR3_IRQn, 3);
    eRcvState = STATE_RX_INIT;
    vRS485PortTimersEnable();
}

void RS485_SetBaudRate(uint32_t baud_rate)
{
	printf("set baud rate:%d\r\n",baud_rate);
	
    UART_DisableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);	

	// reset_buffer((unsigned char *) rs485_rx_buffer , 0x00 , SIZEOF(rs485_rx_buffer ));      
	memset((uint8_t*) rs485_rx_buffer , 0x00 , SIZEOF(rs485_rx_buffer ));           
	vRS485PortClearRXFIFO();
	vRS485PortResetState();

	UART_Close(RS485_PORT); 

    SYS_UnlockReg();
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
	CLK_DisableModuleClock(UART2_MODULE);
    CLK_EnableModuleClock(UART2_MODULE);
    SYS_ResetModule(UART2_RST);

    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk)) |
                (SYS_GPC_MFPL_PC5MFP_UART2_TXD | SYS_GPC_MFPL_PC4MFP_UART2_RXD);	
    SYS_LockReg();

    UART_Open(RS485_PORT, baud_rate);

    // Set RX FIFO Interrupt Trigger Level
    // RS485_PORT->FIFO = (RS485_PORT->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    // RS485_PORT->FIFO &= ~ UART_FIFO_RFITL_Msk;
    // RS485_PORT->FIFO |= UART_FIFO_RFITL_1BYTE;

    UART_EnableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);	 

}

void RS485_tx_process(void)
{
    static uint8_t counter = 0;
    uint8_t i = 0;

    for (i = 0; i < 16 ; i++)
    {
        rs485_tx_packet[i] = i;
    }

    rs485_packet_len = 16;
    rs485_tx_packet[0] = 0x5A;
    rs485_tx_packet[1] = 0x5A;
    rs485_tx_packet[2] = counter;

    rs485_tx_packet[13] = counter;
    rs485_tx_packet[14] = 0xA5;
    rs485_tx_packet[15] = 0xA5;
				
	RS485_StartTxData(rs485_tx_packet , rs485_packet_len);
    counter++;
}


void RS485_rx_process(void)
{
    uint16_t i = 0;

	if (FLAG_RS485_RCV_DATA_FINISH)
	{
		FLAG_RS485_RCV_DATA_FINISH  = 0; 
        #if 1
        for(i = 0 ; i < usRcvBufferPos; i++)
        {

            printf("%c",rs485_rx_buffer[i]);
        }
        #else
		dump_buffer((unsigned char *) rs485_rx_buffer , usRcvBufferPos);	
        #endif
		memset((uint8_t*) rs485_rx_buffer, 0x00 , usRcvBufferPos);   

		// vRS485PortClearRXFIFO();		
		vRS485PortResetState(); 	

		nRTSPin = RECEIVE_MODE;
		UART_EnableInt(RS485_PORT, UART_INTEN_RDAIEN_Msk);	

	}	
}

void RS485_Init(void)
{
	memset((uint8_t*) rs485_tx_packet, 0x00, sizeof(rs485_tx_packet));
    memset((uint8_t*) rs485_rx_buffer, 0x00, sizeof(rs485_rx_buffer));	

    nRTSPin = RECEIVE_MODE;
    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);

    RS485_SetBaudRate(115200);    
    RS485_SetTimeout(217);	// measure 293us to entry expire when last byte xfer

    NVIC_EnableIRQ(UART02_IRQn);
    NVIC_SetPriority(UART02_IRQn, 0);//(1<<__NVIC_PRIO_BITS) - 2

}

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)Reserved.\r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_WDTRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        
        printf("power on from WDT Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_LVRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_LVRF_Msk);
        
        printf("power on from LVR Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_BODRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_BODRF_Msk);
        
        printf("power on from BOD Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_SYSRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_SYSRF_Msk);
        
        printf("power on from System Reset\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_CPULKRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPULKRF_Msk);
        
        printf("power on from CPU Lockup Reset\r\n");
        return FALSE;
    }   
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
    }

    if (FLAG_RS485_TXPACKET_SEND)
    {
        FLAG_RS485_TXPACKET_SEND = 0;
        RS485_tx_process();
    }

    RS485_rx_process();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
                FLAG_RS485_TXPACKET_SEND = 1;
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	

    if(UART_GET_INT_FLAG(RS485_PORT, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_IS_RX_READY(RS485_PORT))
        {      
            prvvUARTRxISR();
        }
    }

    if(UART_GET_INT_FLAG(RS485_PORT, UART_INTSTS_THREINT_Msk))
    {
        prvvUARTTxISR();
    }


}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	dbg_printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	dbg_printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	dbg_printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	dbg_printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	dbg_printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	dbg_printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    dbg_printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    dbg_printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    dbg_printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    dbg_printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    dbg_printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    dbg_printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    dbg_printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    dbg_printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
//    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));


    // CLK_EnableModuleClock(TMR0_MODULE);
  	// CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR3_MODULE);
  	CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(UART2_MODULE);
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk)) |
                (SYS_GPC_MFPL_PC5MFP_UART2_TXD | SYS_GPC_MFPL_PC4MFP_UART2_RXD);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    RS485_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

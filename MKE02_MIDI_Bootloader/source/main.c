#include "MKE02Z4.h"

typedef unsigned char byte;

enum {
	ERR_OK,
	ERR_SERIAL,
	ERR_FORMAT,
	ERR_ID,
	ERR_CSUM,
	ERR_SEQ,
	ERR_FLASH
};

#define SYSEX_DATA_SIZE		32
#define SYSEX_BIT7_SIZE		5

#define SYSEX_START_OFS		0
#define SYSEX_ID0_OFS		(SYSEX_START_OFS + 1)
#define SYSEX_ID1_OFS		(SYSEX_ID0_OFS + 1)
#define SYSEX_PRODUCT_OFS	(SYSEX_ID1_OFS + 1)
#define SYSEX_SEQ_OFS		(SYSEX_PRODUCT_OFS + 1)
#define SYSEX_DATA_OFS		(SYSEX_SEQ_OFS + 1)
#define SYSEX_BIT7_OFS		(SYSEX_DATA_OFS + SYSEX_DATA_SIZE)
#define SYSEX_CSUM_OFS		(SYSEX_BIT7_OFS + SYSEX_BIT7_SIZE)
#define SYSEX_END_OFS		(SYSEX_CSUM_OFS + 1)
#define SYSEX_BUF_SIZE		(SYSEX_END_OFS + 1)


#define SYSEX_START			0xF0
#define SYSEX_ID0			0x00
#define SYSEX_ID1			0x7F
#define SYSEX_END			0xF7

#define SYSEX_PRODUCT		0x21
#define WRITE_BASE_ADDR		0xA000

#define GPIOA_BIT_B5 (1U<<((1*8) + 5))
#define GPIOA_BIT_C2 (1U<<((2*8) + 2))
#define GPIOA_BIT_C3 (1U<<((2*8) + 3))
#define GPIOB_BIT_E1 (1U<<((0*8) + 1))
#define GPIOB_BIT_E2 (1U<<((0*8) + 2))

#define LED_BLUE 	GPIOA_BIT_C2
#define LED_YELLOW 	GPIOA_BIT_C3
#define LED_RED 	GPIOA_BIT_B5

#define LED_ON(b) GPIOA->PSOR = (b)
#define LED_OFF(b) GPIOA->PCOR = (b)

#define SWITCH GPIOB_BIT_E1
#define POWER_LATCH GPIOB_BIT_E2

typedef unsigned char byte;
#define UART0 ((UART_Type *)UART0_BASE)

//*****************************************************************************
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
//*****************************************************************************
__attribute__ ((used,section(".FlashConfig"))) const struct {
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
    unsigned int word4;
} Flash_Config = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFEFFFF};

void ResetISR(void);

__attribute__ ((section(".after_vectors.zzz")))
void IntDefaultHandler(void) {
	for(;;);
}

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
extern void _vStackTop(void);

extern int main(void);


//*****************************************************************************
// The vector table.
// This relies on the linker script to place at correct location in memory.
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
extern void * __Vectors __attribute__ ((alias ("g_pfnVectors")));

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM0P
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
	IntDefaultHandler,                 // The NMI handler
	IntDefaultHandler,                 // The hard fault handler
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
	IntDefaultHandler,                 // SVCall handler
    0,                                 // Reserved
    0,                                 // Reserved
	IntDefaultHandler,                 // The PendSV handler
	IntDefaultHandler,                 // The SysTick handler

    // Chip Level - MKE02Z4
	IntDefaultHandler,   // 16: Reserved interrupt
	IntDefaultHandler,   // 17: Reserved interrupt
	IntDefaultHandler,   // 18: Reserved interrupt
	IntDefaultHandler,   // 19: Reserved interrupt
	IntDefaultHandler,   // 20: Reserved interrupt
	IntDefaultHandler,        // 21: Command complete and error interrupt
	IntDefaultHandler,          // 22: Low-voltage warning
	IntDefaultHandler,          // 23: External interrupt
	IntDefaultHandler,         // 24: Single interrupt vector for all sources
	IntDefaultHandler,   // 25: Reserved interrupt
	IntDefaultHandler,         // 26: Single interrupt vector for all sources
	IntDefaultHandler,         // 27: Single interrupt vector for all sources
	IntDefaultHandler,        // 28: Status and error
	IntDefaultHandler,        // 29: Status and error
	IntDefaultHandler,        // 30: Status and error
	IntDefaultHandler,          // 31: ADC conversion complete interrupt
	IntDefaultHandler,        // 32: Analog comparator 0 interrupt
	IntDefaultHandler,         // 33: FTM0 single interrupt vector for all sources
	IntDefaultHandler,         // 34: FTM1 single interrupt vector for all sources
	IntDefaultHandler,         // 35: FTM2 single interrupt vector for all sources
	IntDefaultHandler,          // 36: RTC overflow
	IntDefaultHandler,        // 37: Analog comparator 1 interrupt
	IntDefaultHandler,      // 38: PIT CH0 overflow
	IntDefaultHandler,      // 39: PIT CH1 overflow
	IntDefaultHandler,         // 40: Keyboard interrupt0
	IntDefaultHandler,         // 41: Keyboard interrupt1
	IntDefaultHandler,   // 42: Reserved interrupt
	IntDefaultHandler,          // 43: Clock loss of lock
	IntDefaultHandler,         // 44: Watchdog timeout
};

//__attribute__ ((section(".after_vectors.zzz")))
void delay(int ms) {
	while(ms--) {
		for(volatile int q = 0; q<2000; ++q) {
			//asm("nop");
		}
	}
}

__attribute__ ((section(".after_vectors.reset")))
void ResetISR(void) {

    // disable watchdog
    __asm volatile ("cpsid i");
    WDOG->CNT = WDOG_UPDATE_KEY1;
    WDOG->CNT = WDOG_UPDATE_KEY2;
    WDOG->TOVAL = 0xFFFFU;
    WDOG->CS1 = (uint8_t) ((WDOG->CS1) & ~WDOG_CS1_EN_MASK) | WDOG_CS1_UPDATE_MASK;
    WDOG->CS2 |= 0;
    __asm volatile ("cpsie i");

    // configure the input line for the switch
	GPIOB->PDDR &= ~(GPIOB_BIT_E1);
	GPIOB->PIDR &= ~(GPIOB_BIT_E1);
	PORT->PUEH |= (GPIOB_BIT_E1);
    delay(10);

    // latch the power on
	GPIOB->PDDR |= GPIOB_BIT_E2;
	GPIOB->PSOR = GPIOB_BIT_E2;

	// is the switch being pressed?
    if(!(GPIOB->PDIR & GPIOB_BIT_E1)) {
    	main();
    }
    else {

    	unsigned int *app_vtor = (unsigned int*)0xA000;

    	unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    	*pSCB_VTOR = (unsigned int)app_vtor;

    	typedef void (*app_ptr_t)(void);
    	app_ptr_t the_app = (app_ptr_t)app_vtor[1];
    	the_app();

    	//unsigned int * pSCB_AIRCR = (unsigned int *) 0xE000ED0C;
    	//*pSCB_AIRCR = (unsigned int)0x05FA0004;
    }
    for(;;);

    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.
//    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
//    if ((unsigned int *)g_pfnVectors!=(unsigned int *) 0x00000000) {
//        *pSCB_VTOR = (unsigned int)g_pfnVectors;
//    }
}


__attribute__ ((section(".after_vectors.zzz")))
void uart_init() {
    SIM->SCGC |= SIM_SCGC_UART0_MASK;
	UART0->BDH = 0;
	UART0->BDL = 40;
    UART0->C2 |= (UART_C2_TE_MASK|UART_C2_RE_MASK); // enable rx/tx
}


__attribute__ ((section(".after_vectors.zzz")))
void error(int code) {
	LED_OFF(LED_BLUE);
	LED_OFF(LED_YELLOW);
	for(;;) {
		for(int i=0; i<code; ++i) {
			LED_ON(LED_RED);
			delay(200);
			LED_OFF(LED_RED);
			delay(200);
		}
		delay(500);
	}
}

__attribute__ ((section(".after_vectors.zzz")))
byte read_byte(byte *ch) {
	while(!(UART0->S1 & UART_S1_RDRF_MASK)) {
		if(UART0->S1 & (UART_S1_OR_MASK|UART_S1_NF_MASK|UART_S1_FE_MASK|UART_S1_PF_MASK)) {
			return 0;
		}
	}
	*ch = UART0->D;
	return 1;
}

//Before launching a command, FSTAT[ACCERR] and FSTAT[FPVIOL] must be cleared
//and the FSTAT[CCIF] flag will be tested to determine the status of the current command
//write sequence. If FSTAT[CCIF] is 0, indicating that the previous command write
//sequence is still active, a new command write sequence cannot be started and all writes to

//__attribute__ ((section(".after_vectors.zzz")))
void fc_init() {
	while(!(FTMRH->FSTAT & 0x80));
}
// set up a parameter for a flash command
//__attribute__ ((section(".after_vectors.zzz")))
void fc_param(uint8_t index, uint8_t hi, uint8_t lo) {
    FTMRH->FCCOBIX = index;
    FTMRH->FCCOBHI = hi;
    FTMRH->FCCOBLO = lo;
}
// run a flash command
//__attribute__ ((section(".after_vectors.zzz")))
void fc_run() {
    __asm volatile ("cpsid i");
	while(!(FTMRH->FSTAT & 0x80));
	FTMRH->FCLKDIV &= 0xe0;
	FTMRH->FCLKDIV |= 0x13; // divide flash clock for 20MHz bus clock
	FTMRH->FSTAT |= 0x30; // clear ACCERR and FPVIOL (write 1 to clear bits!)
	MCM->PLACR |= (1U<<16);
	FTMRH->FSTAT |= 0x80; // kick off the command
	while(!(FTMRH->FSTAT & 0x80)); // wait for CCIF to go high
	MCM->PLACR &= ~(1U<<16);
    __asm volatile ("cpsie i"); // Reenable interrupts

	if(FTMRH->FSTAT & 0x30) {
		error(ERR_FLASH);
	}

}



__attribute__ ((section(".after_vectors.zzz")))
int main(void) {
	/*
	// ICS->C1 = 0x22 = 0b00100010
		7\ CLKS = 00 output of FLL is selected
		6/
		5\
		4| RDIV = 100 /16
		3/
		2  IREFS = 0 external clock
		1  IRCLKEN = 1 IRCLK is active
		0  IREFSTEN = 0 internal ref clock disabled in stop mode

	// ICS->C2 = 0x00
	 	7 \
	 	6 |BDIV 000 divide by 1
	 	5 /
	 	4  LP FLL is not disabled in bypass
	 	3 reserved
	 	2 reserved
	 	1 reserved
	 	0 reserved

	 // OSC->CR = 0xb4  10110100

	    7  OSCEN - 1 enabled
	    6  reserved
	    5  OSCSTEN - 1 enable in stop mode
	    4  OSCOS - 1 osc clock source
	    3  reserved
	    2  RANGE - 1 high freq range
	    1  HGO - 0 low power
	    0  OSCINIT (read 1 when stable)

	 */
    SIM->BUSDIV = 0x01;

    // configure oscillator and wait for settle
    OSC->CR = 0b10110100;
    while(!(OSC->CR & 0x01));

    // configure clock sources
    ICS->C1 = 0x22;
    ICS->C2 = 0x00;

    // wait for FLL to lock
    while (!(ICS->S & 0x40));
    ICS->S |= 0x80; // clear loss of lock flag

    // Initialise GPIO for LEDs and switch
	GPIOA->PDDR |= (GPIOA_BIT_B5|GPIOA_BIT_C2|GPIOA_BIT_C3);

	// Initialise the UART for 31250bps receive
    //SIM->SCGC |= SIM_SCGC_UART0_MASK;
	//UART0->BDH = 0;
	//UART0->BDL = 40;
    //UART0->C2 |= UART_C2_RE_MASK;

    uart_init();

	// set LEDs to initial state
    LED_ON(LED_BLUE);
    LED_ON(LED_YELLOW);
    LED_OFF(LED_RED) ;

	int i;
    volatile int seq_no = 0;
    int addr = WRITE_BASE_ADDR;
    for(;;) {


    	//////////////////////////////////////////////////////////////////////////////////////
    	// read a block of sysex data... the sysex file should be formatted
    	// with an exact number of data blocks
    	//////////////////////////////////////////////////////////////////////////////////////

        byte sysex[SYSEX_BUF_SIZE];
      	for(i = 0; i<SYSEX_BUF_SIZE; ++i) {
      		//byte ch;
      		//if(!read_byte(&ch)) {
      		//	error(ERR_SERIAL);
      		//}
      		//sysex[i] = ch;
      		// wait for a character to be received, or UART error
      		while(!(UART0->S1 & UART_S1_RDRF_MASK)) {
      			 //check whether an error occurred
      			if(UART0->S1 & (UART_S1_OR_MASK|UART_S1_NF_MASK|UART_S1_FE_MASK|UART_S1_PF_MASK)) {
        			error(ERR_SERIAL);
      			}
      		}
      		// store the received byte
      		sysex[i] = UART0->D;
      	}

    	//////////////////////////////////////////////////////////////////////////////////////
      	// Validate the block of data
    	//////////////////////////////////////////////////////////////////////////////////////

      	// check that data is framed by standard MIDI sysex start/end
      	// if not, there is a problem with the format of the file
    	if(sysex[SYSEX_START_OFS] != SYSEX_START ||
    		sysex[SYSEX_END_OFS] != SYSEX_END) {
			error(ERR_FORMAT);
    	}

    	// validate the sysex manufactured ID info and the product
    	// number at the start of the data
    	if(	sysex[SYSEX_ID0_OFS] != SYSEX_ID0 ||
    		sysex[SYSEX_ID1_OFS] != SYSEX_ID1 ||
    		sysex[SYSEX_PRODUCT_OFS] != SYSEX_PRODUCT ) {
			error(ERR_ID);
    	}

    	// Calculate and validate the checksum
    	volatile byte csum = 0;
    	for(i=SYSEX_SEQ_OFS; i<SYSEX_CSUM_OFS; ++i) {
    		csum ^= sysex[i];
    	}
    	if(csum != sysex[SYSEX_CSUM_OFS]) {
			error(ERR_CSUM);
    	}

    	// check for sequence number of zero, which means
    	// end of data
    	if(!sysex[SYSEX_SEQ_OFS]) {

    		// Success indication - need to cycle power to reset
    		for(;;) {
    			LED_ON(LED_BLUE);
    		    LED_OFF(LED_YELLOW);
    			delay(200);
    			LED_OFF(LED_BLUE);
    		    LED_ON(LED_YELLOW);
    			delay(200);
    		}
    	}

    	// Validate the sequence number
    	if(++seq_no>127) {
    		seq_no = 1;
    	}
    	if(sysex[SYSEX_SEQ_OFS] != seq_no) {
			error(ERR_SEQ);
    	}

    	//////////////////////////////////////////////////////////////////////////////////////
      	// Form the full 8 bit data for programming
    	//////////////////////////////////////////////////////////////////////////////////////

    	// MIDI data is 7 bit, so each of the 32 data bits is missing
    	// its top bit. These follow the data bytes, packed into 5
    	// additional bytes. We need to stuff these top bits back in
    	// again...
    	byte mask = 1;
    	int bit7src = SYSEX_BIT7_OFS;
    	for(i=SYSEX_DATA_OFS; i<SYSEX_BIT7_OFS; ++i) {
    		if(sysex[bit7src]&mask) {
    			sysex[i] |= 0x80;
    		}
    		mask<<=1;
    		if(mask&0x80) {
    			mask = 1;
    			++bit7src;
    		}
    	}

    	//////////////////////////////////////////////////////////////////////////////////////
      	// Program the data to FLASH
    	//////////////////////////////////////////////////////////////////////////////////////
    	int src = SYSEX_DATA_OFS;
    	for(i=0; i<4; ++i) {

    		// when we reach the boundary of a new FLASH sector,
    		// we need to erase it (~20ms)
        	if(!(addr&0x1FF)) {
            	fc_init();
            	fc_param(0, 0x0A, 0x00);
    			fc_param(1, addr>>8, (byte)addr);
            	fc_run();
        	}

        	// now program 8 bytes into flash, performing the
        	// neccessary byte re-ordering (~0.2ms)
			fc_init();
			fc_param(0, 0x06, 0x00);
			fc_param(1, addr>>8, (byte)addr);
			fc_param(2, sysex[src+1], sysex[src+0]);
			fc_param(3, sysex[src+3], sysex[src+2]);
			fc_param(4, sysex[src+5], sysex[src+4]);
			fc_param(5, sysex[src+7], sysex[src+6]);
			fc_run();
			src+=8;
			addr+=8;
    	}

    	// toggle the blue LED with each data block
    	if(seq_no & 1) {
    		LED_ON(LED_BLUE);
    	}
    	else {
    		LED_OFF(LED_BLUE);
    	}
    }


    return 0 ;
}

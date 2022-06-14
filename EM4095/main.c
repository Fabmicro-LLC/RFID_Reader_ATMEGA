// *********************************************** 
// project : RFID Reader V2.0
// Target : MEGA8-16AU
// Crystal: 14.7456 Mhz
// Input : EM4095 RFID 125KHz
// Output : RS232C 115200,N,8,1
// Author : leeky@avrtools.co.kr
// Adapted by: rz@fabmicro.ru
// *********************************************** 

// *********************************************** 
// Pin configuration for ATMEGA8-16AU TQFP32
// *********************************************** 
// #30 PD0/RXD <--- RXD (RS232C)
// #31 PD1/TXD ---> TXD (RS232C)
// # 9 PD5/T1 <--- TIMER1 EXT-T1 <--- RDY-CLK (EM4095)
// #10 PD7 ---> SHD (EM4095)
// #11 PD6 ---> MOD (EM4095)
// #12 PB0/ICAP <--- TIMER1 ICAP <--- DEMOD (EM4095)
// #13 PB1 ---> LED (Indicator)
// #14 PB2 ---> BUZZER (Warning)
// #15 PB3 ---> RELAY (Output)
// *********************************************** 
#define F_CPU 14745600	// oscillator-frequency in Hz
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

void fill_rf_buff(char num_byte);

// *********************************************** 
// Variables for System 
// *********************************************** 
volatile unsigned int adc_buff;
volatile unsigned char rxd_buff;
 
// *********************************************** 
// Initialize Variables for flags
// *********************************************** 
volatile unsigned char flag_1ms =0;    // timer 1 ms	 
volatile unsigned char flag_adc =0;	 	// end of adc conversion
volatile unsigned char flag_rxd =0;	 	// serial received
volatile unsigned char flag_err =0;	 	// header error
volatile unsigned char parity_err =0;	// parity error

// *********************************************** 
// Constant define
// *********************************************** 
#define REL 0x08		// Relay =PB3																  
#define BUZ 0x04		// Buzzer =PB2																  
#define LED 0x02		// indicator =PB1
#define DEM 0x01 		// RF Data input EXT-INT0 from PB0/ICAP

#define RXD 0x01		// PD0/RXD <--- ROUT SP232 <--- PC TXD  
#define TXD 0x02 		// PD1/TXD ---> TINP SP232 ---> PC RXD
#define CLK 0x20		// EXT-T1 input for RF read clock =PD5
				// External interrupt used to read clock

#define SHD 0x80 		// RF shot down for RF disable =PD7
#define MOD 0x40		// RF 100% modulation =PD6

// *********************************************** 
// Global variables for buffer
// *********************************************** 
volatile unsigned char rf_buff[16];	// rf buffer  
volatile char rf_buff_ptr =0;		// rf buffer pointerÍ
volatile char rf_bit_ptr = 0;		// rf bit pointer

// *********************************************** 
// Global variables
// *********************************************** 
volatile char bit_value =1;			// received bit value
volatile char bit_saved =1;			// flag of next bit will be store  
volatile char edge_dir =1;			// Edge Direction is riging										 

volatile char stable_width =0;			// RF uncertainty
volatile char bit_trans_num =0;			// number of bit stored

volatile int old_width =0;			// old width of captured timer1 value
volatile int timer_over =0;			// flag of time overflow 

// *********************************************** 
// Initialize Ports
// *********************************************** 
void port_init(void)
{
	PORTB = 0x01;	// PB3=Relay,PB2=Buzzer,PB1=LED,
	DDRB  = 0xFE;	// PB7~PB1 =output,PB0=DEMOD input. 
 
 	PORTC = 0x00;	// PORTC = adc input (not used) 
 	DDRC  = 0x00;	// PC7~PC0 = all input

 	PORTD = 0xCF;	// PD6=MOD,PD7=SHD,PD5=RDY-CLK(input)
 	DDRD  = 0xC0;	// PD7~PD6 =output
}

// eeprom_init()
// Initialize internal eeprom
void eeprom_init(void)
{
	EECR = 0x00;	// Disable interrupts
}

// *********************************************** 
// ADC initialisation
// Conversion time: 104uS
// *********************************************** 
void adc_init(void)
{
  ADCSR = 0x00; 	   //disable adc
  ADMUX = 0x07; 	   //select adc input 7 only
  ACSR  = 0x80;
  ADCSR = 0xEF;
}

// *********************************************** 
// Interrupt handler for ADC
// conversion complete, read value (int) using...
// value=ADCL; Read 8 low bits first (important)
// value|=(int)ADCH << 8; 
// read 2 high bits and shift into top byte
// *********************************************** 
ISR(ADC_vect)
{
  adc_buff =ADCL;	 	  	  			// get low byte only
  adc_buff |=(int)ADCH << 8;  	// get high byte and amke int
  flag_adc =1;	   	  	  			// end of adc convert
}

// *********************************************** 
// TIMER2 initialisation - prescale:128
// WGM: Normal
// desired value: 1mSec
// actual value:  0.998mSec (0.2%)
// *********************************************** 
void timer2_init(void)
{
 	TCCR2 = 0x00; 			//stop
 	ASSR  = 0x00; 			//set async mode
 	TCNT2 = 0x8D; 			//setup
 	TCCR2 = 0x05; 			//start
	TIMSK |= 0x40; 			// timer2 interrupt sources
}

// *********************************************** 
// Interrupt handler for TIMER2 1ms overflow
// *********************************************** 
ISR(TIMER2_OVF_vect)
{
	TCNT2 = 0x8D; 		// reload counter value
	flag_1ms =1;	  	// flag_1ms =1
	timer_over++;		// 1ms timer +1
}

// *********************************************** 
// Delay for ms
// *********************************************** 
void delay_ms(int dly)
{
 	 unsigned int i;
	 for (i=0; i <dly; i++)		// 1ms *N =N [ms]	
	 {
		 while (flag_1ms ==0){}		// Stand by 1ms
		 flag_1ms =0; 	 				// reset 1 ms timer		  
	 } 
}
 
// *********************************************** 
// UART initialize for 115200,N,8,1
// char size: 8 bit, parity: Disabled
// *********************************************** 
#define BAUD_RATE 57600	// x2 =115,200 bps
#define BAUD_CALC ((F_CPU)/((BAUD_RATE)*16l)-1)
// 9273599

void uart0_init(void)
{
	UCSRB = 0x00; //disable while setting baud rate
	UCSRA = 0x02; //double baud rate
	// 0x86 = asynchronous 8N1
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
	// set baud rate
	UBRRH =BAUD_CALC >>8;
	UBRRL =BAUD_CALC;
	// 0x90 = Enable receiver and transmitter
	UCSRB = (1<<RXEN)|(1<<TXEN);
	// Enable RX IRQ
	//UCSRB = (1<<RXCIE);
}


void uart_putchar(uint8_t c, FILE *stream) {
        loop_until_bit_is_set(UCSRA, UDRE); /* Wait until data register empty. */
        UDR = c;
        UCSRA |= (1<<TXC); // Clear Transfer complete bit by writing 1 to it
}


int uart_getchar(uint8_t *c) {

    if(! bit_is_clear(UCSRA,RXC))  {
        *c = UDR;
        return 1;
    } else
        return 0;
}


static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
//static FILE uart_inpuT = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
//static FILE uart_io   = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


// *********************************************** 
// TIMER1 initialze for ICP cature 
// timer1 clock is EXT_T1 from RDY-CLK of EM4095
// PB0/ICP capture input is DEMOD sinal from EM4095
// *********************************************** 
void timer1_init(char edge)
{
	TCCR1B = 0x00; 			// stop timer1 
	TCNT1H = 0x00;
	TCNT1L = 0x00;			// start from 0000

	ICR1H  = 0x00;
	ICR1L  = 0x00;			// using timer1 capture

	TCCR1A = 0x00;
	TCCR1B = 0x00;			// stop timer1, 

	if (edge ==1)
		TCCR1B = 0x40;  // ICAP capture from T1 riging.
	else 
		TCCR1B = 0x20;	// ICAP capture from T1 falling. 

	TIMSK |= 0x24; 			// timer1 capture, overflow interrupt sources
}

// *********************************************** 
// TIMER1 overflow interrupt
// *********************************************** 
ISR(TIMER1_OVF_vect)
{
  //TIMER1 has overflowed
 	TCNT1H = 0x00;				 	// reload counter high value
 	TCNT1L = 0x00;				 	// reload counter low value
}

// *********************************************** 
// Received bit store by buffer pointer in rf buffer
// *********************************************** 
void rf_bit_store(int bit_value)
{
	unsigned char byte;

	if(rf_buff_ptr < sizeof(rf_buff)) { 	// not end of buffer?
		byte = rf_buff[rf_buff_ptr];	// get saved byte by pointer
		byte = (byte << 1);		// shift left
		
		byte |= (bit_value & 0x01);

		rf_buff[rf_buff_ptr] = byte;	// store received bit

 		if(++rf_bit_ptr == 8) {		// end of bit pointer?
			rf_bit_ptr = 0;	 	// bit pointer =0
   			++rf_buff_ptr;		// increament byte pointer 
 		}
	}
}

// *********************************************** 
// PB0/ICP capture interrupt 
// captured 16 bit ICR1 register read 1st low byte
// current edge direction chage to reverse edge direction
// *********************************************** 
ISR(TIMER1_CAPT_vect)
{
	unsigned int value;		// temporary for catured value
	unsigned int width;		// temporary for width calc. 

	value = ICR1L;			// Read low byte first (important)
	value |=(int)ICR1H << 8; 	// Read high byte and shift into top byte

	if(edge_dir) {			// change ICP capture direction of edge. 
		TCCR1B &= ~0x40;
		TCCR1B |= 0x20;		 // ICP capture direction edge to riging 
		edge_dir =0;		 // for next cature direction is falling.    
	} else {
		TCCR1B &= ~0x20;
		TCCR1B |= 0x40;		 // ICP capture direction edge to falling 
		edge_dir =1;		 // for next capture direction is riging
	}

	width = value - old_width;	// width = new captured width - old captured width
	old_width = value;		// updata old captured width for next width calc.
	
	// Receiving manchester code from RFID card. 
	// 1st received bit must be 1 for it is header after reset, 
	// if pulse width is more than 40 then received bit to invert.

 	if(width > 40) {		// narrow pulse width is under 32,
		if (bit_value) PORTB |=LED;	// debug indictor for input bits
		else PORTB &= ~LED;

		bit_value = ~bit_value; 	// received bit is inverting
		rf_bit_store(bit_value);	// save inverted bit
		++bit_trans_num;		// increament number of saved bits
  		bit_saved = 0;  		// skip bit store when bit inverted
	}
		
	if(bit_saved) {        		// will be bit store?
		if (bit_value) PORTB |=LED;	// debug indictor for input bits 
		else PORTB &= ~LED;
				
		rf_bit_store(bit_value);	// bit store in rf buffer
		++bit_trans_num;		// number of stored bits +1
	}
	bit_saved = ~bit_saved;  		// skip next store for next bit 
}

// *********************************************** 
// RFID Card Reader EM4095 initialize
// EM4095 output ---> PD5/T1 =RDY-CLK, PB0/ICP =DEMOD
// EM4095 input <--- PD6 =MOD, PD7 =SHD.
// *********************************************** 
void em4095_init()
{
	//PORTB |= BUZ;					// Buzzer =on 	 
	PORTB |= LED;					// LED =on
	delay_ms(100);					// delay 100ms

	//PORTB &= ~BUZ;					// Buzzer =off 	 
	PORTB &= ~LED;					// LED =off
	delay_ms(100);					// delay 100ms

	PORTD |= SHD;					// EM4095 shut down												 
	PORTD |= MOD;					// EM4095 max outup
	PORTB |= LED;					// LED =on
	delay_ms(100);					// delay 100ms
	 
	PORTD &= ~SHD;					// EM4095 ready
	PORTD &= ~MOD;					// EM4095 min output
	PORTB &= ~LED;					// LED =off
	delay_ms(100);					// delay 100ms
}

// *********************************************** 
// Change direction of caturing edge 
// by PB0/ICP input signal from DEMOD output of EM4095
// *********************************************** 
void set_rf_edge(char edge)
{
	if(edge) {
		timer1_init(1);		// setup timer1 to rising edge
		edge_dir =1;		// status =rf_fe_toggle on
	} else {
		timer1_init(0); 	// setup timer1 to falling edge
		edge_dir =0;		// status =rf_fe_toggle off
	}
}

// *********************************************** 
// Receiving 64 bits stream from RFID card at Timer1 capture
// output is number of saved bit and received 64 bis in rf buffer
// *********************************************** 
void get_bits_num(char num_bits, char edge)
{
	TCCR1B = 0x00;			// stop Timer1 from EXT-T1 

	PORTD &= ~SHD;			// turn on EM4095 
	PORTD |= MOD;			// turn on modulation (RF is low amplitude) 
	delay_ms(40);			// Wait for EM4095 to wakeup
//	PORTD |=MOD;			// turn on modulation (transmit power to transponder) 
	PORTD &= ~MOD;			// Turn off modulation (trasmit power to transponder) 
//	delay_ms(20);			// turn off time =20 ms,

	fill_rf_buff(16);		// erase rf buffer 16 bytes. 
	set_rf_edge(edge);		// change direction of caturing edge

	// force save flag =1 for must save 1st received bit.
	bit_saved = 1;      					 

	// If captured width is more than 40 then bit to inverting
	bit_value = 0;								// bit value =0

	rf_buff_ptr = 0;             // byte pointer =0
	rf_bit_ptr = 0;             	// bit pointer =0
	bit_trans_num = 0;        	// number of save bit =0
	timer_over = 0;             	// time over value =0
	old_width = 0;								// old width =0

	// Reinitialize timer1
 	TCNT1H = 0x00;				 							
 	TCNT1L = 0x00;							// tiemr0 =0x0000¤
	TCCR1B = 0x07;							// start timer1  
	TIMSK |= 0x20; 							// enable timer1 capture interrupt

//	PORTD &= ~MOD;			// Turn off modulation and start reading 

	// If number of saved bit is less than num_bits then receiving bit
	while(bit_trans_num < num_bits  && timer_over < 60) { } 
	// If number of saved bit is more than num_bits 
	// or time is more than 60ms then exit receiving bit 
	TIMSK &= ~0x20; 						// stop timer1 capture interrupt

	rf_buff_ptr = 0; 						// reset buffer pointer¤
	rf_bit_ptr = 0;             // reset bit pointer

	//if(timer_over >= 60) 
	//	printf("get_bits_num() insufficient data received, bit_trans_num = %d, num_bits = %d\n", bit_trans_num, num_bits);

	//printf("get_bits_num() bit_trans_num = %d, num_bits = %d\n", bit_trans_num, num_bits);

	PORTD |=SHD;			// turn off EM4095 
}

// *********************************************** 
// erase all bits in rf buffer
// *********************************************** 
void fill_rf_buff(char num_byte)
{
	int i;

	rf_buff_ptr =0;
	rf_bit_ptr =0;
	 
	for(i = 0; i < num_byte; i++)
		rf_buff[i] = 0x00;
}

// *********************************************** 
// Inverting all bits in rf buffer
// *********************************************** 
void invert_rf_buff(void)
{
	int i;

	rf_buff_ptr =0;
	rf_bit_ptr =0;

	for(i=0; i < sizeof(rf_buff); i++)
		rf_buff[i] = ~rf_buff[i];
}

// *********************************************** 
// get bit from rf buffer
// Output is TRUE or FALSE (0x01 or 0x00)
// *********************************************** 
int get_buff_bit(void)
{
	int bit;
	unsigned char byte;
	
	byte = rf_buff[rf_buff_ptr];
	byte =(byte << rf_bit_ptr);			// bit shift for 0~7
	byte &=0x80;
		
	if (byte ==0x80) 
		bit =0x01;
	else 
		bit =0x00;			 			// make 1 bit result
						
	if(++rf_bit_ptr == 8) {						// end of bit pointer?
		rf_bit_ptr = 0;						// bit pointer =0
		rf_buff_ptr++;	 	 				// byte pointer +1
	}

	return bit;			
}

// *********************************************** 
// gte byte from received bufferÙ
// *********************************************** 
char get_buff_byte(void)
{
  char i;				 		 					// bit counter
  char bit;			 							// get bit
	char byte =0;	 							// get byte
		
  if(!(rf_buff_ptr ==sizeof(rf_buff)))
  {
    for(i=0; i<8; ++i)
    {
		 	bit =get_buff_bit();			// get bit from buffer.

			if (bit) byte |=0x01;			// bit =1®
			else byte &=0xfe;					// bit =0Â

    	byte =(byte <<1);			 	 	// bit shift to left for make byte®
		}
  }
 	return byte;
}

// *********************************************** 
// Find 9 bits header.
// *********************************************** 
void find_header(void)
{
	int i;		// bit counter
	int bit;	// get bit
	
	rf_buff_ptr =0;
	rf_bit_ptr =0; 		// start of buffer
	flag_err =0;		// reset flag of error		

	// for 9 bits test
	for(i =0; i < 9; i++) {
		bit =get_buff_bit();
		if (bit ==0) {
			flag_err =1;
			break;
		}
	}
}

// *********************************************** 
// Check horizontal parity.
// *********************************************** 
void checksum_h(void)
{
	char j;			// nibble counter
	char k;			// bit counter
	char bit;		// get bit
	char parity;								
	
	parity_err =0; 		// reset parity error 
	for (j=0; j < 10; j++)			// for 10 of nibbles
	{
	  parity =0;								// result =0 for parity calulationÙ
		for(k =0; k < 4; k++)			// for 4 of bits. 
		{
  	  bit =get_buff_bit();		// get bit for 0x00 or 0x01.
			if (bit) parity ^=0x01;	// calculate parity for 4 bits.
		}

 		bit =get_buff_bit();			// 5th bit is parity.
		if (bit !=parity) parity_err =1;	// not match is found error.
	}
}

// *********************************************** 
// Check vertical parity.
// *********************************************** 
void checksum_v(void)
{

}

// *********************************************** 
// 64 bits ASCII transmit for binary dispaly.			
// *********************************************** 
void tx_buff_bin(void)
{
  char bit;
	char txd;
	char i,j;
	 
	rf_buff_ptr =0;
	rf_bit_ptr =0;
	uart_putchar(0x0a, NULL);	 					// lf =start of text

	for (i =0; i <64; i++)
	{
    bit =get_buff_bit();			// get bit from rf_buff
		if (bit) txd =0x31;				// 0x31 =ascii '1'
		else txd =0x30;						// 0x30 =ascii '0'
		uart_putchar(txd, NULL);						// '0' or '1' to uart
	}
	uart_putchar(0x0d, NULL);	 					// cr =end of text
}
	
// *********************************************** 
// 10 digit ASCII transmit to UART for hexa dispaly.			
// *********************************************** 
void tx_buff_hex(void)
{
	char bit;
	unsigned char txd;
	char i,j;
	 
	rf_buff_ptr =1;							// skip 9 bits of header¡
	rf_bit_ptr =1;							

	printf("RFID: ");

	for (i = 0; i < 10; i++) {			// 10 nibbles
		for (j = 0; j < 4; j++) {		// per 4 bits each
			txd = (txd << 1); 		// shift to left for saved bit
			txd |= get_buff_bit();		// get bit from rf_buff
		}

		// to HEX
		txd &= 0x0F;	 		
		if (txd < 0x0A) 
			txd += '0';		// add '0'
		else 
			txd = txd - 0x0A + 'A';		// add 'A'
		
		uart_putchar(txd, NULL);	// ASCII transmit to UART
		bit = get_buff_bit();		// read and skip parity bit
	}
	printf("\n");
}

// *********************************************** 
// Main routine 
// *********************************************** 
void main(void)
{
	cli(); 	   			   					//disable all interrupts
	//MCUCR = 0x00;
	//GICR  = 0x00;

	port_init();
	eeprom_init();		  				// disable eeprom interrupt
	adc_init(); 			  			// 10 bit adc #7 (free run)

	timer2_init();		  				// init 10us timer
	uart0_init();			  	 		// init serial port
	sei();  	   			  	 	// re-enable interrupts

        stdout = &uart_output;
        //stdin  = &uart_input;

	_delay_ms(1000);
	printf("EM4095 RFID Card Reader V2.0\n");

	em4095_init();						 	// init EM4095 RFID reader
	timer1_init(0);				 			// init timer1 from EXT-T1 (EM4095-CLK)
	fill_rf_buff(16);			  		// erase rf buffer. 


	while(1) {

		static int counter = 0;

		if((counter++ % 10) == 0) {
			printf("EM4095 IS READY\n");
		}

		//PORTD |=MOD;						// turn off EM4095 
		//PORTD |=SHD;						// turn off EM4095 
		//delay_ms(20);						// turn off time =20 ms,
		//fill_rf_buff(16);			  		// erase rf buffer 16 bytes. 
		//PORTD &= ~MOD;					// start EM4095 read mode
		//PORTD &= ~SHD;						// start EM4095 read mode

		// EM type (Read only) data format
		// 1111 11111 = Header = 9 bit­
		// xxxxP, xxxxp= Custom #1,#2  = 10 bit
		// xxxxP,xxxxP = data #1,#2  = 10 bit
		// xxxxP,xxxxP = data #3,#4  = 10 bit
		// xxxxP,xxxxP = data #5,#6  = 10 bit
		// xxxxP,xxxxP = data #7,#8  = 10 bit
		// PPPP0 = 4 Parity + 1 stop = 5 bit
		// data 55 bits + header 9 bits =64 bits®
		
		// Manchester code detecting method
		// Header (9 bits)
		// ______--__--__--__--__--__--__--__--__-- (wave)
		// 0-0 0-1 0-1 0-1 0-1 0-1 0-1 0-1 0-1 0-1  (edge)
		// (x) (1) (1) (1) (1) (1) (1) (1) (1) (1)  (code)
		
		// Custom-ID (10 bits) 
		// --__--__--__--__--__--____----__--____-- (wave)
		// 1-0 1-0 1-0 1-0 1-0 1-0 0-1 1-0 1-0 0-1  (edge)
		// (0) (0) (0) (0) (0) (0) (1) (0) (0) (1)  (code)
		//  D7  D6  D5  D4  Pr  D3  D2  D1  D0  Pr  (Pr =parity)
		      
		// find manchester pattern in received bits
		// output is number of received bits =bit_trans_num
		get_bits_num(64, 0);	 			// receive 64 bits stream within 60ms 	

		if(timer_over >= 60)
			goto timeout;

		printf("RAW DATA: ");
		for(int i = 0; i < 16; i++)
			printf("%02X ", rf_buff[i]);
		printf("\n");

		PORTB &= ~LED;	 	 					// Indicator turn off

		find_header();							// check head is 9 bit =1?
		checksum_h();							// check horizontal parity
		checksum_v();							// check vertical parity
		
		if (flag_err ==0) {						// has no head error?
			//printf("9-bit header found!\n");
			if (parity_err == 0) {					// has no parity error? 
				//printf("Valid KEY found\n");
				// tx_buff_bin();				// 64 binary display code to UART.
				tx_buff_hex();					// 10 hexa dispaly code to UART.
		  		//PORTB |=REL;					// turn on relay.
			} else {
				//PORTB |=BUZ;					// if error available then turn on buzzer.		
			}
		}												

		timeout:
		// Total cycle time = reste 20ms + receive 60ms + output 120ms = 200msâ
		delay_ms(60);						// output time = 120ms
		//delay_ms(20);						
		//PORTB &= ~BUZ;							// turn off buzzer.
		//PORTB &= ~REL;							// turn off relay.
	}	
}



#define F_CPU 14745600UL	// 14.7456 Mhz

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mfrc522.h"

//#define ABS(X)  (X >= 0 ? X : X * -1)
static int32_t ABS(int32_t X) {
        if(X < 0) return -1*X;
        return X;
}


#define ubrrval(BAUD)		(((F_CPU)/(BAUD*16UL))-1) 
#define	DEBUG_BAUDRATE		115200UL
#define	UART_READ_TIMEOUT_MS	10


#define RX_BUF_SIZE     128
#define TX_BUF_SIZE     128

#define SQRT(X) ((uint32_t)sqrt((double)X))

static uint16_t my_build_number = BUILD_NUMBER;

volatile uint16_t timer_counter0 = 0;
volatile uint16_t timer_counter1 = 0;
volatile uint16_t timer_counter2 = 0;
volatile uint8_t timer_counter3 = 0;
uint32_t syscounter = 0;
static uint8_t rxbuf[RX_BUF_SIZE+1];
static uint8_t txbuf[TX_BUF_SIZE+1];
volatile int16_t rx_index = 0;

volatile uint8_t flag_1ms = 0;

void delay_ms(int ms)
{
	int i;

	for(i = 0; i < ms; i++) {
		while (flag_1ms == 0); /* Stand by 1ms */
		flag_1ms = 0;
	}
}

uint16_t crc16(uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
  crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

  for (int i = 8; i != 0; i--) {    // Loop over each bit
    if ((crc & 0x0001) != 0) {      // If the LSB is set
      crc >>= 1;                    // Shift right and XOR 0xA001
      crc ^= 0xA001;
    }
    else                            // Else LSB is not set
      crc >>= 1;                    // Just shift right
    }
  }

  return crc;
}

void add_crc(uint8_t *buf, uint16_t len)
{
	uint16_t crc = crc16(buf, len);
	buf[len] = (crc & 0xff);
	buf[len+1] = (crc >> 8) & 0xff;
}


uint8_t gpio_get_bit(volatile uint8_t* port, uint8_t bit)
{
	uint8_t mask = 0x01 << bit;

	return (*port & mask) >> bit;
}

void gpio_set_bit(volatile uint8_t* port, uint8_t bit, uint8_t val)
{
	uint8_t mask = 0x01 << bit;

	if(val) {
		*port |= mask; // ON
	} else {
                *port &= ~ mask; // OFF 
	}

}

void gpio_toggle_bit(volatile uint8_t* port, uint8_t bit)
{
	uint8_t mask = 0x01 << bit;

	*port ^= mask;

}

void hardware_init(void)
{
	uint16_t i;

	PORTA = 0x0; // PA = 0 - Shutdown MFRC522 
	DDRA = 1; // PA0 - Output: MFRC522_RESET

	DDRD = 0x00; // PD0 - Input: MFRC522_RQ, PD7 - Input - MFRC522_MX
	PORTD = 0x00; // No pull 

	DDRD = 0x08; // PF3 - Output: LED_WORK
	PORTD = 0xf7; // PullUP 

	// Timer1/B is for system time ticks = 32768 Hz
	TCNT1H = 0x00; // Initial value for Timer1
	TCNT1L = 0x00; // Initial value for Timer1
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10); 
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); // CS=1/1, WGM=0100 CTC,  

	// For 8MHz
	//OCR1AH = 0x00; // Match reg high byte
	//OCR1AL = 0xff; // Match reg low byte

	// For 14.7MHz
	OCR1AH = 0x01; // Match reg high byte
	OCR1AL = 0xC0; // Match reg low byte

	TIMSK = (1<<OCIE1A); // Enable interrupts from Timer1 compare match A

	// WatchDogTimer 2.2sec
	WDTCR=0x1F;
	WDTCR=0x0F;
}

void usart0_init(void)
{
	//Disable receiver and transmitter
	UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0));

	UBRR0H=ubrrval(DEBUG_BAUDRATE)>>8; 
	UBRR0L=ubrrval(DEBUG_BAUDRATE);

	UCSR0C =(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00); // 8N1
	//UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
	UCSR0B &= ~(1 << RXCIE0); // Disable the USART Recieve Complete interrupt (USART_RXC)

	//Enable The receiver and transmitter
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}


void usart1_init(void)
{

	//Disable receiver and transmitter
	UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1));

	// 9600
	UBRR1H=ubrrval(9600UL)>>8; 
	UBRR1L=ubrrval(9600UL);

	// 115200
	//UBRR1H=ubrrval(115200UL)>>8; 
	//UBRR1L=ubrrval(115200UL);

	UCSR1C =(1<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10); // 8N1
	//UCSR1B |= (1 << RXCIE1); // Enable the USART Recieve Complete interrupt (USART_RXC)
	UCSR1B &= ~(1 << RXCIE1); // Disable the USART Recieve Complete interrupt (USART_RXC)

	//Enable The receiver and transmitter
	UCSR1B |= (1<<RXEN1)|(1<<TXEN1);
}

void usart0_putchar(uint8_t c, FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
	UDR0 = c;
	UCSR0A |= (1<<TXC0); // Clear Transfer complete bit by writing 1 to it
}

void usart1_putchar(uint8_t c, FILE *stream)
{
	loop_until_bit_is_set(UCSR1A, UDRE1); /* Wait until data register empty. */
	UDR1 = c;
	UCSR1A |= (1<<TXC1); // Clear Transfer complete bit by writing 1 to it
}


int usart0_getchar(uint8_t *c)
{
	if(! bit_is_clear(UCSR0A,RXC0))  {
		*c = UDR0;
		return 1;
	} else
		return 0;
}

int usart1_getchar(uint8_t *c)
{
	int i;

	for (i = UART_READ_TIMEOUT_MS; i > 0; i--)
		if (! bit_is_clear(UCSR1A,RXC1))
			break;
		else
			delay_ms(1);

	if(i == 0)
		return 0;
 
	*c = UDR1;
	return 1;
}

static FILE uart_output = FDEV_SETUP_STREAM(usart0_putchar, NULL, _FDEV_SETUP_WRITE);
//static FILE uart_inpuT = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
//static FILE uart_io 	= FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void main(void)
{	
	int i, rc;

	hardware_init();
	usart0_init(); // Debug output port
	usart1_init(); // MFRC522 port
	stdout = &uart_output;
	//stdin  = &uart_input;

	sei(); // Enable interrupts

	delay_ms(50); // Wait for clocks to settle

	printf("\r\nRFIDReader_MFRC522, build number #%04d.\r\nCopyright (C) 2022, Fabmicro LLC.\r\n", my_build_number);

	// Init MFRC522 chip

	rc = mfrc522_Init();	

	printf("MFRC522_INFO: Initialization %s, rc = %d\r\n", rc == 0 ? "OK":"FAIL", rc);

        while(1)
        {
		__asm__ __volatile__ ("wdr"); // reset WatchDog Timer


		if(timer_counter2 > 4096) {

			// 8 Hz timer 
			timer_counter2 = 0;
			syscounter++;

			/* Scan RF for PICC devices */
			rc = mfrc522_PICC_IsNewCardPresent();

			if(rc == 0) {
				Uid uid;
				memset(&uid, 0, sizeof(uid));
				rc = mfrc522_PICC_Select(&uid, 0 /* validBits, if set to 0 no uid supplied */);

				uint8_t type = mfrc522_PICC_GetType(uid.sak);
				const char *type_name = mfrc522_PICC_GetTypeName(type);

				if(rc == 0) {

					rc = mfrc522_PICC_HaltA();

					printf("MFRC522_INFO: Card %02X%02X%02X%02X type %d present: %s\r\n", 
						uid.uidByte[0],uid.uidByte[1],uid.uidByte[2],uid.uidByte[3],
						type, type_name);
					printf("MFRC522_DEBUG: HaltA rc: %d, uid.size: %d\r\n", rc, uid.size);
				} else {
					printf("MFRC522_INFO: Error %d reading card type %d: %s\r\n", 
						rc, type, type_name);
				}
			} else { 
				//printf("MFRC522_INFO: No card present, rc: %d\r\n", rc);
			}

		}

		if(timer_counter0 > 32768) {

			// 1 Hz timer 
			timer_counter0 = 0;

			gpio_toggle_bit(&PORTF, PF3);


			printf("MFRC522_READY: syscount: %08ld, build: %d\r\n", syscounter, my_build_number);

			if(rx_index > 0) {
				for(i = 0; i < rx_index; i++)
					printf("MFRC522_RX[%d]: 0x%02X\r\n", i, rxbuf[i]); 
				rx_index = 0;
			}

		}
        }

}


ISR(TIMER1_COMPA_vect) {
	cli();
	timer_counter0++;
	timer_counter1++;
	timer_counter2++;
	timer_counter3++;

	if(timer_counter3 == 32) {
		flag_1ms = 1;
		timer_counter3 = 0;
	}

	sei();
}


ISR(USART0_RX_vect) {
	
	// Debug port input

printf("XXX\r\n");
	/* Do nothing
	char a;
	cli();
	/usart0_getchar(&a); // just skip input data on RX0
	sei();
	*/
}


ISR(USART1_RX_vect) {
	
	// MFRC522 port input

	/* Do nothing
	cli();
	// ...	
	sei();

	*/
}


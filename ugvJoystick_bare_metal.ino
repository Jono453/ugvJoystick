/*

Driver to send vintage joystick inputs to 915 MHz telemetry link
to send to RC car

References:
http://www.built-to-spec.com/blog/2009/09/10/using-a-pc-joystick-with-the-arduino/
https://old.pinouts.ru/InputCables/GameportPC_pinout.shtml
https://baremetalmicro.com/tutorial_avr_digital_io/04-Outputs.html

*/

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define USART_BAUDRATE 57600
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ASYNCHRONOUS (0<<UMSEL00) // USART Mode Selection
#define DISABLED    (0<<UPM00)
#define EVEN_PARITY (2<<UPM00)
#define ODD_PARITY  (3<<UPM00)
#define PARITY_MODE  DISABLED // USART Parity Bit Selection

#define ONE_BIT (0<<USBS0)
#define TWO_BIT (1<<USBS0)
#define STOP_BIT ONE_BIT      // USART Stop Bit Selection

// UCSZ00 controls data size
#define DATA_BIT   (3<<UCSZ00)  // USART Data Bit Selection

void adc_init()
{ 
    // ADEN is the ADC Enable bit
    // ADIE is the ADC Interuupt Enable bit
    // ADPS is for prescaler. at 12 MHz, prescaler set to 128
    // 12 Mhz / 128 = 93.7kHz
    ADCSRA = (1 << ADEN)| ( 1 << ADIE) | (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);
    ADMUX |= (1 << REFS0); //Reference set to 01 which is Vcc
    sei(); //global interrupt

    ADCSRA |= (1 << ADSC); //start ADC conversion. Initial ADC conversion takes longest
}

ISR(ADC_vect)
{  

  // Convert data stored in ADCL and ADCH registers 
  switch (ADMUX)
  {
    case 0xC0:    
      ADMUX |= (1 << REFS0) | (1 << MUX0); //ADC1 switch. 2 bytes     
      // print ADCW
      break;
      
    case 0xC1:      
      ADMUX |= (1 << REFS0); //ADC0 switch
      // print ADCW
      break;
      
    default:
      break;
  } 
  ADCSRA |= (1 << ADSC); // not using auto-trigger yet
}

uint16_t adc_read(uint8_t ch)
{
  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
 
  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);
}

void uart_init(void) 
{

  // Set Baud Rate
  UBRR0H = BAUD_PRESCALER >> 8; //load upper 8bits of prescaler
  UBRR0L = BAUD_PRESCALER; //load lower 8bits of prescaler
  
  // Set Frame Format
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;
  
  // Enable Receiver and Transmitter
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void USART_TransmitPolling(uint8_t DataByte)
{
  while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
  UDR0 = DataByte; //USART data register
}


void gpio_setup()
{
    // Port D Data Direction Register (DDRD)
    DDRD |= (1 << PIND7) | (1 << PIND6); // D7 and D6 as an output pins
    DDRD &= ~(1 << PIND2); //D2 as input pin
}

int main(void)
{

  gpio_setup();  
  adc_init();
  uart_init();
  sei();

  char buffer[10];

  // Loop forever
  while (1)
  {
      // Toggle D6 LED based on D2 joystick input (make interrupt later)
      // For safety switch
      if (PIND & (1 << PIND2))
      {        
        PORTD &= ~(1 << PIND6);
      }
      else
      {
        PORTD |= (1 << PIND6);        
 
        // Port D Data Register (PORTD)
        // Set PD7 
        PORTD |= (1 << PIND7);        

        USART_TransmitPolling(adc_read(0));
        
        // Wait 250ms
        _delay_ms(250);        

        // Set pins low
        PORTD &= ~(1 << PIND7);

        // Wait 250ms
        _delay_ms(250);   
      }
  }

}

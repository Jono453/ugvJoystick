/*

Driver to send vintage joystick inputs to 915 MHz telemetry link
to send to RC car

References:
http://www.built-to-spec.com/blog/2009/09/10/using-a-pc-joystick-with-the-arduino/
https://old.pinouts.ru/InputCables/GameportPC_pinout.shtml
https://baremetalmicro.com/tutorial_avr_digital_io/04-Outputs.html

*/

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // Port D Data Direction Register (DDRD)
    DDRD |= (1 << PIND7) | (1 << PIND6); // D7 and D6 as an output pins
    DDRD &= ~(1 << PIND2); //D2 as input pin

    // Loop forever
    while (1)
    {

        // Toggle D6 LED based on D2 joystick input (make interrupt later)
        // For safety switch
        if (PIND & (1 << PIND2))
        {
          PORTD |= (1 << PIND6);        
        }
        else
        {
          PORTD &= ~(1 << PIND6);
        }        

        // Port D Data Register (PORTD)
        // Set PD7 
        PORTD |= (1 << PIND7);        

        // Wait 250ms
        _delay_ms(250);

        // Set pins low
        PORTD &= ~(1 << PIND7);

        // Wait 250ms
        _delay_ms(250);

    }
}

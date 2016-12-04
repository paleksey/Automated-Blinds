/*
 * AutomatedWindowShades.c
 *
 * Created: 12/3/2016 3:49:21 PM
 * Author : Group 13
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int ElapsedFourSeconds = 0; // New counter variable

// variables for automated shades state machine 
typedef enum {open, closed} current_state = open;
typedef enum {yes, no} light, previous_light;
typedef enum {open, close, stay_put} blinds;
unsigned int hits = 0;

#define AND &&
#define OR ||
	
	
	typedef enum { false, true } bool;

int main(void)
{
	
	
	// SET UP TIMER INTERRUPTS (FOR POLLING THE LDR)
	TCCR1B |= (1 << WGM12);                                // Configure timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A);                               // Enable CTC interrupt
	
	
	// CONFIGURE THE ADC (FOR READING THE LDR)
	ADMUX |= 1 << REFS0;                                   // Set AVcc as the reference voltage for the ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable a prescaler
	// ADMUX |= 1 << ADLAR;                                   // 8-bit or 10-bit results
	// ADCSRA = 1 << ADIE                                     // Enable interrupts function in ADC
	ADCSRA |= 1 << ADEN;                                   // Turn on the ADC feature
	
	
	// ENABLE GLOBAL INTERRUPTS
	sei();                                                 
	
	
	// FINISH TIMER INTERRUPTS (FOR LDR POLLING)
	OCR1A = 62500;                                         // Set CTC compare value to 1 KHz at 1 MHz AVR clock, with prescaler of 1024
	TCCR1B |= ((1 << CS10) | (1 << CS12));                 // Start timer at F_cpu/1024
	
    /* Replace with your application code */
    while (1) 
    {
    }
	
}


ISR(TIMER1_COMPA_vect) {
	
	int light;
	
	// Keeps track of four seconds passing
	ElapsedFourSeconds++;
	
	// check if 2 minutes (120 seconds) have elapsed
	if (ElapsedFourSeconds == 30) {
		
		ElapsedFourSeconds = 0;  // Reset counter variable
		
		// take a reading of the LDR sensor
		ADCSRA |= 1 << ADSC; // start the first conversion
		
		// we are synchronously waiting for the conversion to complete
		while(!(ADCSRA & (1<<ADIF)));
		
		// compare ADC to set value
		if (ADC >= 523) {
			light = yes;
		} else {
			light = no;
		}
		
		// figure out number of hits
		if (previous_light == light) {
			if (hits < 5) {
				hits++;
			}
		} else {
			hits = 0;
		}
		previous_light = light;
		
		// determine state machine output (what should the blinds do next)
		if (current_state == closed AND light == yes AND hits == 5) {
			current_state = open;
			blinds = open;
		} else if (current_state == open AND light == no AND hits == 5) {
			current_state = closed;
			blinds = close;
		} else {
			current_state = current_state;
			blinds = stay_put;
		}
		
		// Clear ADIF by writing one to it
		ADCSRA|=(1<<ADIF);
		
	}
	
}

//#if DLEVEL > 5
//#define SIGNAL  1
//#if STACKUSE == 1
//#define STACK   200
//#else
//#define STACK   100
//#endif
//#else
//#define SIGNAL  0
//#if STACKUSE == 1
//#define STACK   100
//#else
//#define STACK   50
//#endif
//#endif
//#if DLEVEL == 0
//#define STACK 0
//#elif DLEVEL == 1
//#define STACK 100
//#elif DLEVEL > 5
//display( debugptr );
//#else
//#define STACK 200
//#endif

//#ifdef DEBUG
//printf("debug:x = %d, y = %f\n", x, y);
//...
//#endif


// questions we have left:
// what happens if we are interrupted when we are polling


// test if polling block triggers correctly every X time
// test with a LED at each point
// make sure my custom variables work
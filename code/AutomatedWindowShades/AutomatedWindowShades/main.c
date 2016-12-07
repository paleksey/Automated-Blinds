/*
 * AutomatedWindowShades.c
 *
 * Created: 12/3/2016 3:49:21 PM
 * Author : Group 13
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
	#include <stdint.h>


#define AND &&
#define OR ||


// VARIABLES FOR AUTOMATED SHADES STATE MACHINE 
typedef enum {light, dark} outside_conditions;
outside_conditions current_state = light;

typedef enum {yes, no} light_bool;
light_bool light_now;
light_bool light_before = no;

unsigned int hits = 0;

unsigned int ElapsedFourSeconds = 0; // New counter variable

int ADCval;

volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

volatile uint32_t max_height = 0;
volatile uint32_t current_height = 0;


int main(void)
{
	
	
	// SET UP TIMER INTERRUPTS (FOR POLLING THE LDR)
	TCCR1B |= (1 << WGM12);                                // Configure timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A);                               // Enable CTC interrupt
	

	// CONFIGURE THE ADC (FOR READING THE LDR)
		//int ADCval;
	ADMUX |= 1 << REFS0;                                   // Set AVcc as the reference voltage for the ADC
		ADMUX &= ~(1 << ADLAR);		// Clear for 10 bit resolution
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable a prescaler
		ADCSRA |= (1 << ADEN);		// Enable the ADC
	// ADMUX |= 1 << ADLAR;                                   // 8-bit or 10-bit results
	// ADCSRA = 1 << ADIE                                     // Enable interrupts function in ADC
	//ADCSRA |= (1 << ADEN);                                   // Turn on the ADC feature
	

	// CONFIGURE THE INTERRUPTS FOR THE BUTTONS
	PCICR |= (1 << PCIE1);                               // Turn on pin interrupts for PD pins
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT11);              // Mask Interrupts for only the pins you need


	// ENABLE GLOBAL INTERRUPTS
	sei(); 


	// FINISH TIMER INTERRUPTS (FOR LDR POLLING)
	OCR1A = 62500;                                         // Set CTC compare value to 1 KHz at 1 MHz AVR clock, with prescaler of 1024
	TCCR1B |= ((1 << CS10) | (1 << CS12));                 // Start timer at F_cpu/1024

	
	// set all LEDs as outputs
	DDRD |= (1 << DDD2) | (1 << DDD1) | (1 << DDD0);
	
	// make LEDs all high to disable them
	PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
	
    /* Replace with your application code */
    while (1) 
    {
    }
	
}





ISR(PCINT1_vect) {
	
	uint8_t changedbits;

	changedbits = PIND ^ portbhistory;
	portbhistory = PIND;	
	
	// check if programming button is pressed
	if (changedbits & (1<< PC3)) {
		
		// disable interrupts
		
		// reset variable -> now we are assuming shades are at the top
		current_height = 0;
		
		// force the servo to move down
		// servo(down);
		
		// start keeping track of servo position
		while (PC3 == 0) {
			current_height = current_height + 1;
		}
		
		// update the max height so now we know where bottom is
		max_height = current_height;
		
		// turn the interrupts back on
		sei();
		
		
		
	// check if up button is pressed
	} else if (changedbits & (1 << PC1)) {
		
		// disable interrupts
		
		// check position of blinds
		if (current_height > 0) {
			//servo(move_up);
			while (PC1 == 0 && current_height > 0) {
				current_height = current_height - 1;
			} 
			// servo(stop);
		}
		
		/* PCINT0 changed */
		// enable interrupts
		sei();
	}
		
	
}





void turnOnLeds(int color) {

	// make LEDs all high to disable them
	PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
	
	
	// red
	if (color == 0) {
			PORTD &= ~(1 << PD0); //  turn on PD1 (PD1 is grounded)	
			
	// blue
	} else if (color == 1) {
		PORTD &= ~(1 << PD1); //  turn on PD1 (PD1 is grounded)	
		
	// green
	} else if (color == 2) {
		PORTD &= ~(1 << PD2); //  turn on PD1 (PD1 is grounded)	
	// white 
	} else if (color == 3) {
		PORTD &= ~(1 << PD2);
		PORTD &= ~(1 << PD1);
		PORTD &= ~(1 << PD0);
	} else {
		// do nothing
	}
	
}




ISR(TIMER1_COMPA_vect) {
	
	// Keeps track of four seconds passing
	ElapsedFourSeconds++;
	//DDRD |= (1 << DDD0) ; // sets bit DDD0 to 1 within register DDRD (PD0 is now an output)
	//PORTD &= ~(1 << PORTD0); //  turn off PD0 (PD0 is grounded)
	//PORTD = PORTD ^ 0x01;	// Toggle the RGB
	
	// check if 2 minutes (120 seconds) have elapsed
	if (ElapsedFourSeconds == 1) {
		
		ElapsedFourSeconds = 0;  // Reset counter variable
		
		// Take a reading of the LDR sensor
		ADCSRA |= 1 << ADSC; // start the first conversion

		//// we are synchronously waiting for the conversion to complete
		while(ADCSRA & (1<<ADIF));
		//
		// compare ADC to set value
		if (ADC >= 523) {
			light_now = yes;
		} else {
			light_now = no;
		}

		// figure out number of hits
		if (light_before == light_now) {
			if (hits < 2) {
				hits++;
			}
		} else {
			hits = 0;
		}
		light_before = light_now;
		
		// determine state machine output (what should the blinds do next)
		if (current_state == dark AND light_now == yes AND hits == 2) {
			current_state = light;
			turnOnLeds(2);
			// aleskeyfunction(2);
		} else if (current_state == light AND light_now == no AND hits == 2) {
			current_state = dark;
			turnOnLeds(1);
			// aleskesyfunction(0);
		} else {
			turnOnLeds(3);
			current_state = current_state;
		}
		
		// Clear ADIF by writing one to it
		// Clearing ADC bit
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
// what if something happens, when something else is happening
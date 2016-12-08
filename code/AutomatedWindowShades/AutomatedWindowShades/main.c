/*
 * AutomatedWindowShades.c
 *
 * Created: 12/3/2016 3:49:21 PM
 * Author : Group 13
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 10 MHz clock speed
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
	#include <stdint.h>
#include <util/delay.h>


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

volatile uint8_t portbhistory = 0x7F;     // default is high because the pull-up

volatile uint32_t max_height = 0;
volatile uint32_t current_height = 0;


uint8_t changedbits; // make not global later

typedef enum {stop, up, down, callibrate} button;
button buton_up = up;
button buton_stop = stop;
button buton_down = down;
button buton_callibrate = callibrate;

void servo(button action);
void turnOnLeds(int color, int toggle);


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
	

	// CONFIGURE THE BUTTONS
	DDRC &= ~(1 << PC1) | ~(1 << PC2) | ~(1 << PC3)| ~(1 << PC6);  // make the up button an input (clear bit)
	PORTC |= (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC6);  // make the up button tied high (set bit)

	// CONFIGURE THE INTERRUPTS FOR THE BUTTONS
	PCICR |= (1 << PCIE1);                               // Turn on pin interrupts for PD pins
	//PCMSK1 |= (1 << PCINT9) | (1 << PCINT11);              // Mask Interrupts for only the pins you need
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT11) | (1 << PCINT10); 

	// ENABLE GLOBAL INTERRUPTS
	sei(); 


	// FINISH TIMER INTERRUPTS (FOR LDR POLLING)
	OCR1A = 62500;                                         // Set CTC compare value to 1 KHz at 1 MHz AVR clock, with prescaler of 1024
	// TCCR1B |= ((1 << CS10) | (1 << CS12));                 // Start timer at F_cpu/1024

	
	// set all LEDs as outputs
	DDRD |= (1 << DDD2) | (1 << DDD1) | (1 << DDD0);
	
	// make LEDs all high to disable them
	PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
	
	// servo(up);
	//turnOnLeds(3, 0);
	
    /* Replace with your application code */
    while (1) 
    {
		//PORTD = PORTD ^ 0x04;	// Toggle the RGB
		//_delay_ms(500);
    }
	
}




void servo(button action)
	{

	    if(action == callibrate)
	    {
	        DDRB  |= (1 << DDB1); // set output to PB1
			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = 3999;  //set ICR1 to produce 50Hz frequency
			  OCR1A = 3699;   // 3999 * 0.925 most left
	    }
		    
	    if(action == down)
	    {
	       DDRB  |= (1 << DDB1); // set output to PB1

			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = 3999;  //set ICR1 to produce 50Hz frequency
			  OCR1A = 3659;   // 3999 * 0.925 most left
	    }
	    
	    if(action == up)
	    {
	      DDRB  |= (1 << DDB1); // set output to PB1
			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = 3999;  //set ICR1 to produce 50Hz frequency
			  OCR1A = 3739;   // 3999 * 0.935 most left
	    }

		if(action == stop)
		{
			DDRB &= ~(1 << DDB1);
			TCCR1A &= (0<<COM1A1) & (0<<COM1A1);
		}
			    
	}
	


ISR(PCINT1_vect) {
	
	
	changedbits = PINC ^ portbhistory;
	portbhistory = PINC;	

	
	// check if programming button is pressed
	if (~PINC & (1 << PC3)) {
		
		turnOnLeds(3, 1);
		
		// disable interrupts
		cli();
		
		// reset the height, we are assuming shades are at the top
		current_height = 0;
		
		// force the servo to move down
		// servo(down);
		
		// start keeping track of servo position
		while (~PINC & (1 << PC3)) {
			turnOnLeds(1, 0);
			current_height = current_height + 1;
		}
		
		// stop the servo
		// servo(stop);
		
		// update the max height so now we know where bottom is
		max_height = current_height;
		
		turnOnLeds(0, 0);
		
		// turn the interrupts back on
		sei();
		
	// check if up button is pressed
	} else if (~PINC & (1 << PC1)) {
		
		turnOnLeds(0, 1);
		
		// disable interrupts
		cli();
		
		// check position of blinds
		if (current_height > 0) {
			
			// start moving the window shades up
			//servo(move_up);
			
			while ((~PINC & (1 << PC1)) && current_height > 0) {
				turnOnLeds(1, 0);
				current_height = current_height - 1;
			} 
			
			// stop moving the window shades
			// servo(stop);
			
			turnOnLeds(2, 0);
			
		}
		
		// enable interrupts
		sei();
		
	} else if (changedbits & (1 << PC2)) {
		turnOnLeds(2, 1);
	}
		
	
}





void turnOnLeds(int color, int toggle) {

	// red
	if (color == 0) {
		if (toggle) {
			PORTD = PORTD ^ 0x01;	// Toggle the RGB	
			//PIND = _BV(PD0);
		}
		else {
			// make LEDs all high to disable them
			PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
			
			PORTD &= ~(1 << PD0); //  turn on PD1 (PD1 is grounded)	
		}
	// blue
	} else if (color == 1) {
		if (toggle) {
			PORTD = PORTD ^ 0x02;	// Toggle the RGB
		}
		else {
			// make LEDs all high to disable them
			PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
			
			PORTD &= ~(1 << PD1); //  turn on PD1 (PD1 is grounded)	
		}
	// green
	} else if (color == 2) {
		if (toggle) {
			PORTD = PORTD ^ 0x04;	// Toggle the RGB	
		}
		else {
			// make LEDs all high to disable them
			PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
			
			PORTD &= ~(1 << PD2); //  turn on PD1 (PD1 is grounded)	
		}
	// white 
	} else if (color == 3) {
		if (toggle) {
			PORTD = PORTD ^ 0x07;	// Toggle the RGB
		}
		else {
			// make LEDs all high to disable them
			PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
			
			PORTD &= ~(1 << PD2);
			PORTD &= ~(1 << PD1);
			PORTD &= ~(1 << PD0);
		}
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
			turnOnLeds(2, 0);
			// aleskeyfunction(2);
		} else if (current_state == light AND light_now == no AND hits == 2) {
			current_state = dark;
			turnOnLeds(1, 0);
			// aleskesyfunction(0);
		} else {
			turnOnLeds(3, 0);
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

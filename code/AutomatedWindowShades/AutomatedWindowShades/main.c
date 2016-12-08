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


unsigned int IR_state = 0;

// VARIABLES FOR AUTOMATED SHADES STATE MACHINE 
typedef enum {light, dark} outside_conditions;
outside_conditions current_state = light;

typedef enum {yes, no} light_bool;
light_bool light_now;
light_bool light_before = no;

unsigned int hits = 0;

unsigned int ElapsedFourSeconds = 0; // New counter variable

int ADCval;

volatile uint8_t portbhistory;     // default is high because the pull-up

volatile uint32_t max_height = 0;
volatile uint32_t current_height = 0;


uint8_t changedbits; // make not global later

typedef enum {stop, up, down, callibrate} button;
button buton_up = up;
button buton_stop = stop;
button buton_down = down;
button buton_callibrate = callibrate;

volatile int programmed = 0;

void servo(button action);
void turnOnLeds(int color, int toggle);


int main(void)
{
	
	
	// SET UP TIMER INTERRUPTS (FOR POLLING THE LDR)
	TCCR0B |= (1 << WGM01);                                // Configure timer 1 for CTC mode
	

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
	DDRC &= ~(1 << PC1) | ~(1 << PC2) | ~(1 << PC3)| ~(1 << PC6) | ~(1 << PC5) | ~(1 << PC4);  // make the up button an input (clear bit)
	PORTC |= (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC6) | (1 << PC5);  // make the up button tied high (set bit)

	// CONFIGURE MODE SWITCH
	portbhistory = PINC;


	// CONFIGURE THE INTERRUPTS FOR THE BUTTONS
	PCICR |= (1 << PCIE1);                               // Turn on pin interrupts for PD pins
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT11) | (1 << PCINT10) | (1 << PCINT13) | (1 << PCINT12); 

	// ENABLE GLOBAL INTERRUPTS
	sei(); 


	// FINISH TIMER INTERRUPTS (FOR LDR POLLING)
	OCR0B = 250;                                         // Set CTC compare value to 1 KHz at 1 MHz AVR clock, with prescaler of 1024
	TCCR0A |= ((1 << CS02) | (1 << CS00));                 // Start timer at F_cpu/1024

	
	// set all LEDs as outputs
	DDRD |= (1 << DDD2) | (1 << DDD1) | (1 << DDD0);
	
	// make LEDs all high to disable them
	PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);

	// switch to automatic mode
	if (PINC & (1 << PC4)) {
		turnOnLeds(4, 0);
		TIMSK0 |= (1 << OCIE0B);                               // Enable CTC interrupt
	// switch to manual mode
	} else {
		turnOnLeds(0, 0);
		TIMSK0 &= ~(1 << OCIE0B);                               // Enable CTC interrupt
	}

	
	// servo(up);
	
    /* Replace with your application code */
    while (1) 
    {
		//PORTD = PORTD ^ 0x04;	// Toggle the RGB
		//_delay_ms(500);
    }
	
}




void servo(button action)
	{
		
		int PERIOD =  ((16000000*8/46.5)-1);
		int CALIBRATE_TIMER =  (PERIOD / (20 + 1.5) * 20);
		int UP_TIMER =  (PERIOD / (20 + 1.7) * 20);
		int DOWN_TIMER =  (PERIOD / (20 + 1.3) * 20);
		

	    if(action == callibrate)
	    {
	        DDRB  |= (1 << DDB1); // set output to PB1
			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = PERIOD;  //set ICR1 to produce 50Hz frequency
			  OCR1A = CALIBRATE_TIMER;   // 42552 * 0.925 most left
	    }
		    
	    if(action == down)
	    {
	       DDRB  |= (1 << DDB1); // set output to PB1

			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = PERIOD;  //set ICR1 to produce 50Hz frequency
			  OCR1A = DOWN_TIMER;   // 42552 * 0.925 most left
	    }
	    
	    if(action == up)
	    {
	      DDRB  |= (1 << DDB1); // set output to PB1
			  TCCR1A = ((1 << COM1A0) | (1 << COM1A1) | (1 << WGM11)); // Inverting + WGM mode 14 
			  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11)); // WGM mode 14 (Fast PWM), and 8x prescaler
			  //(16000000 / 8 / 40000 = 50hz)
			  ICR1  = PERIOD;  //set ICR1 to produce 50Hz frequency
			  OCR1A = UP_TIMER;   // 42552 * 0.935 most left
	    }

		if(action == stop)
		{
			DDRB &= ~(1 << DDB1);
			TCCR1A &= (0<<COM1A1) & (0<<COM1A1);
		}
			    
	}
	


ISR(PCINT1_vect) {
	// PCICR |= (1 << PCIE1);                               // Turn on pin interrupts for PD pins
	
	changedbits = PINC ^ portbhistory;
	portbhistory = PINC;	
	
	

	// PCICR &= ~(1 << PCIE1); 
	PCMSK1 &= ~(1 << PCINT13);
	TIMSK0 &= ~(1 << OCIE0B);                               // Enable CTC interrupt
	

	
	// check if programming button is pressed
	if (~PINC & (1 << PC3)) {
		
		// disable interrupts
		cli();
		
		// reset the height, we are assuming shades are at the top
		current_height = 0;
		
		// force the servo to move down
		servo(down);
		
		turnOnLeds(2, 0);
		
		// start keeping track of servo position
		while ((~PINC & (1 << PC3)) AND (programmed == 0 OR current_height >= 0)) {
			current_height = current_height + 1;
		}
		
		// stop the servo
		servo(stop);
		
		// update the max height so now we know where bottom is
		max_height = current_height;
		
		// we have now programmed the device
		programmed = 1;
		
		turnOnLeds(-1, 0);
		
		// turn the interrupts back on
		sei();
		
	// check if up button is pressed
	} else if (~PINC & (1 << PC1)) {
		
		// disable interrupts
		cli();
		
		// check position of blinds
		if (current_height > 0 OR programmed==0) {
			
			// start moving the window shades up
			servo(up);
			
			turnOnLeds(3, 0);
			
			//while ((~PINC & (1 << PC1))) {
			while ((~PINC & (1 << PC1)) AND (programmed == 0 OR current_height > 0)) {
				current_height = current_height - 1;
			} 
			
			// stop moving the window shades
			servo(stop);
			
			turnOnLeds(-1, 0);
			
		}
		
		// enable global interrupts
		sei();
	
	
	// down button
	} else if (~PINC & (1 << PC2)) {
		
		// disable interrupts
		cli();
		
		// check position of blinds
		if (current_height < max_height OR programmed == 0) {
			
			// start moving the window shades down
			servo(down);
			
			turnOnLeds(1, 0);
			
			//while ((~PINC & (1 << PC2))) {
			 while ((~PINC & (1 << PC2)) AND (programmed == 0 OR current_height < max_height)) {
				current_height = current_height + 1;
			} 
			
			// stop moving the window shades
			servo(stop);
			
		}
		
		// enable interrupts
		sei();
		
		turnOnLeds(-1, 0);
	
	// mode selection switch
	} else if (changedbits & (1 << PC4)) {
		
		// switch to automatic mode
		if (PINC & (1 << PC4)) {
			turnOnLeds(4, 0);
			TIMSK0 |= (1 << OCIE0B);                               // Enable CTC interrupt
		// switch to manual mode
		} else {
			turnOnLeds(0, 0);
			TIMSK0 &= ~(1 << OCIE0B);                               // Enable CTC interrupt
		}
		
	// see if IR detected
	} else if (~PINC & (1 << PC5)) {
		
		// cli();
		
		
		int x = 5;
		
		
		
		
		
		for (int i = 0; i < 2; i++) {
			
			// last button push was up
			if (IR_state == 0) {
				
				// if shades are already not at the bottom
				if (current_height != max_height) {
					
					turnOnLeds(1, 0);
					
					//_delay_ms(2000);
					
					turnOnLeds(-1, 0);
					
					current_height = current_height + 1;
					
					// start moving the shades down
					servo(down);
					
					// wait until the shades reach the bottom
					while (current_height < max_height AND programmed == 1) {
					// while ((~PINC & (1 << PC5)) && current_height < max_height) {
						current_height = current_height + 1;
					}
					
					// stop the servo
					servo(stop);
					
					// update state
					IR_state = 1;
					break;
				} else {
					IR_state = 1;
				}
			}
			
			// last button push was down
			if (IR_state == 1) {
				// if shades are not already at the top
				if (current_height != 0) {
					
										turnOnLeds(2, 0);
					
					//_delay_ms(2000);
					
					current_height = current_height - 1;
					
					turnOnLeds(-1, 0);
					
					// start moving the shades up
					servo(up);

					
					// wait until the shades reach the top			
					while (current_height > 0 AND programmed == 1) {
					// while ((~PINC & (1 << PC5)) && current_height > 0) {
						current_height = current_height - 1;
					}
					// stop the servo
					servo(stop);
					
					
					// update the state
					IR_state = 0;
					break;
				} else {
					IR_state = 0;
				}
			}
			
		}
		
		
		
		
		
		
		
		
		
		
		
		// PORTC &=~(1<<PC3); //clear PC3
		
		// Disable IR interrupt
		// PCMSK1 &= ~(1 << PCINT13);
		
		//_delay_ms(4000);
	
		//int x;
		// cli();
		
		turnOnLeds(-1, 0);
		
		// PCMSK1 |= (1 << PCINT13);
		
	}
	
			// switch to automatic mode
		if (PINC & (1 << PC4)) {
			turnOnLeds(4, 0);
			TIMSK0 |= (1 << OCIE0B);                               // Enable CTC interrupt
		// switch to manual mode
		} else {
			turnOnLeds(0, 0);
			TIMSK0 &= ~(1 << OCIE0B);                               // Enable CTC interrupt
		}
	
	PCMSK1 |= (1 << PCINT13);

	// PCICR |= (1 << PCIE1); 
	
	
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
		
	// turquoise
	} else if (color == 4) {
		if (toggle) {
			PORTD = PORTD ^ 0x06;	// Toggle the RGB
		}
		else {
			// make LEDs all high to disable them
			PORTD |= (1 << PD2) | (1 << PD1);
			
			PORTD &= ~(1 << PD2);
			PORTD &= ~(1 << PD1);
		}
	} else if (color == -1) {
		// turn off LEDs
		PORTD |= (1 << PD2) | (1 << PD1) | (1 << PD0);
		// do nothing
	}
	
}




ISR(TIMER0_COMPB_vect) {
	
	// Keeps track of four seconds passing
	ElapsedFourSeconds++;
	
	// check if 2 minutes (120 seconds) have elapsed
	if (ElapsedFourSeconds == 20000) {
		
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
			
			turnOnLeds(2, 0);
			
			current_state = light;
			
			// start moving the shades up
			servo(up);
					
			// wait until the shades reach the top			
			while (current_height > 0 AND programmed == 1) {
			// while ((~PINC & (1 << PC5)) && current_height > 0) {
				current_height = current_height - 1;
			}
					
			// stop the servo
			servo(stop);
					
		} else if (current_state == light AND light_now == no AND hits == 2) {
			
			turnOnLeds(1, 0);
			
			current_state = dark;

			// start moving the shades down
			servo(down);
					
			// wait until the shades reach the bottom
			while (current_height < max_height AND programmed == 1) {
			// while ((~PINC & (1 << PC5)) && current_height < max_height) {
				current_height = current_height + 1;
			}
					
			// stop the servo
			servo(stop);
					
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

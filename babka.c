/*
 */
// Choose your MCU here, or in the build script
#define __AVR_ATtiny13A__

#define F_CPU 4800000UL
#define EEPSIZE 64
#define V_REF REFS0
#define BOGOMIPS 950
#define STAR2_PIN   PB0
#define STAR3_PIN   PB4
#define STAR4_PIN   PB3
#define PWM_PIN     PB1
#define VOLTAGE_PIN PB2
#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x05    // clk/32 (makes it 150kHz)
#define PWM_LVL     OCR0B   // OCR0B is the output compare register for PB1

#define FAST 0x23           // fast PWM channel 1 only
#define PHASE 0x21          // phase-correct PWM channel 1 only

// configuration byte bits usage:
// 0 .. 4 - actual level group (for normal mode / bike mode)
// 5 - memory on/off
// 6 .. 8 - so far not used
#define DEFAULTS_CONFIG 0b00000000  // NORMAL mode, 6 levels (level group 0), without memory

// state memory byte bits usage:
// used when memory is ON, to save actual state of flashlight - which mode and which level is set
// 1 .. 2 - active mode (values 0-3: 0=normal, 1=special, 2=ramping mode, 3=bike mode)
// 3 .. 6 - active level from group (only 0-15) | or precise level for ramping mode (by 1/16th of 100%)
// 7 .. 8 - not used yet
#define DEFAULTS_STATE 0b00000000
//#define MEMORY_SAVE 1

#define VOLTAGE_MON		 // get monitoring functions from include
#define USE_BATTCHECK	   // Enable battery check mode
#define BATTCHECK_8bars	 // up to 8 blinks
#include "tk-voltage.h"

#define PWM_RAMP_SIZE  8
#define PWM_RAMP_VALUES   5, 26, 64, 85, 128, 169, 192, 255  // 1, 10, 25, 33, 50, 66, 75, 100%

// These need to be in sequential order, and numbered above PWM_RAMP_SIZE, no gaps.
// Make sure to update FIRST_BLINKY and LAST_BLINKY as needed.
#define FIRST_BLINKY 240
#define BATT_CHECK 240
#define STROBE	241
#define BEACON 242
#define LAST_BLINKY 242

#define MODE_NORMAL 0
#define MODE_BLINKY 1
#define MODE_RAMPING 2
#define MODE_BIKE 3
#define LAST_NORMAL_MODE MODE_BIKE

#define CONFIGURATION_MODE LAST_NORMAL_MODE + 1
#define GROUP_SELECT_MODE CONFIGURATION_MODE + 1

#define CONFIG_BLINK_BRIGHTNESS	2 // output to use for blinks on battery check (and other modes) = 10%
#define CONFIG_BLINK_SPEED		 750 // ms per normal-speed blink

#define TURBO_MINUTES 1 // when turbo timer is enabled, how long before stepping down
#define TICKS_PER_MINUTE 120 // used for Turbo Timer timing
#define TURBO_LOWER 128  // the PWM level to use when stepping down
#define ID_TURBO PWM_RAMP_SIZE	// Convenience code for turbo mode


// Calibrate voltage and OTC in this file:
#include "tk-calibration.h"

/*
 * =========================================================================
 */

// Ignore a spurious warning, we did the cast on purpose
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <string.h>

#define OWN_DELAY		   // Don't use stock delay functions.
#define USE_DELAY_MS	 // use _delay_ms()
#define USE_DELAY_S		 // Also use _delay_s(), not just _delay_ms()
#include "tk-delay.h"

#define NUM_FP_BYTES 3
uint8_t fast_presses[NUM_FP_BYTES] __attribute__ ((section (".noinit")));

// Modes (gets set when the light starts up based on saved config values)
PROGMEM const uint8_t pwm_ramp_values[]  = { PWM_RAMP_VALUES };

#define NUM_LEVEL_GROUPS 8 // Can define up to 16 groups
PROGMEM const uint8_t level_groups[] = {
	1, 2, 4, 5, 7, 8, 0,
	3, 5, 7, 8, 0,
	4, 6, 8, 0,
	1, 0,
	2, 0,
	5, 0,
	1, 2, 3, 0,
	3, 7, 0
};
// has to be the same lenght as the longest level group (in our case 6)
#define LONGEST_LEVEL_GROUP 6 // dont forget to update if editing groups

register uint8_t actual_level_id asm("r6");
register uint8_t actual_mode asm("r7");
register uint8_t actual_pwm_output asm("r8");
register uint8_t config asm("r9");
register uint8_t status asm("r10");

uint8_t available_levels[LONGEST_LEVEL_GROUP];
uint8_t num_available_levels;

// =========================================================================

inline uint8_t config_level_group_number() { return (config     ) & 0b00001111; }
inline uint8_t config_memory_is_enabled()  { return (config >> 4) & 0b00000001; }

inline uint8_t status_mode()     { return (status     ) & 0b00000011; }
inline uint8_t status_level_id() { return (status >> 2) & 0b00001111; }

int main(void) __attribute__((OS_main));

inline void reset_state() {
	config = DEFAULTS_CONFIG;
	status = DEFAULTS_STATE;

	actual_mode = status_mode();
	actual_level_id = status_level_id();
}

inline void next_mode() {
	actual_level_id++;
	
	if(fast_presses[0] == 3 && actual_level_id <= num_available_levels) {  // triple-tap from a solid mode
		if( blinkies_are_enabled() ) actual_level_id = FIRST_BLINKY; // if blinkies enabled, go to first one
		else if( battcheck_is_enabled() ) actual_level_id = BATT_CHECK; // else if battcheck enabled, go to it
	}
	
	// if we hit the end of the solid modes or the blinkies (or battcheck if disabled), go to first solid mode
	if ( (actual_level_id == num_available_levels) || (actual_level_id > LAST_BLINKY) || (actual_level_id == BATT_CHECK && !battcheck_is_enabled() )) {
		actual_level_id = 0;
	}
	
}

inline void count_modes() {
	uint8_t group = 0, mode, i, mc=0;
	uint8_t *dest = available_levels;
	const uint8_t *src = level_groups;
	for(i=0; i<sizeof(level_groups); i++) {
		mode = pgm_read_byte(src+i);
		// if we hit a 0, that means we're moving on to the next group
		if (mode==0) { group++; } 
		// else if we're in the right group, store the mode and increase the mode count
		else if (group == modegroup_number()) { *dest++ = mode; mc++; } 
	}
	num_available_levels = mc;
}

inline void set_output(uint8_t pwm1) {
	PWM_LVL = pwm1;
}

void set_level(uint8_t level) {
	if (level == 1) { TCCR0A = PHASE; } 
	set_output(pgm_read_byte(pwm_ramp_values  + level - 1));
}

void blink(uint8_t val, uint16_t speed)
{
	for (; val>0; val--)
	{
		set_level(CONFIG_BLINK_BRIGHTNESS);
		_delay_ms(speed);
		set_output(0);
		_delay_ms(speed);
		_delay_ms(speed);
	}
}

inline void toggle_mode(uint8_t value, uint8_t num) {
	blink(num, CONFIG_BLINK_SPEED/4);  // indicate which option number this is
	uint8_t temp = actual_level_id;
	actual_level_id = value;
	save_state();
	blink(32, 500/32); // "buzz" for a while to indicate the active toggle window
	
	// if the user didn't click, reset the value and return
	actual_level_id = temp;
	save_state();
	_delay_s();
}

void toggle_options(uint8_t value, uint8_t num) {
	blink(num, CONFIG_BLINK_SPEED/4);  // indicate which option number this is
	uint8_t temp = config;
	config = value;
	save_state();
	blink(32, 500/32); // "buzz" for a while to indicate the active toggle window
	
	// if the user didn't click, reset the value and return
	config = temp;
	save_state();
	_delay_s();
}

inline uint8_t we_did_a_fast_press() {
	uint8_t i;
	for(i=0; i<NUM_FP_BYTES-1; i++) { if(fast_presses[i] != fast_presses[i+1]) return 0;}
	return 1;
}

inline void increment_fast_presses() {
	uint8_t i;
	for(i=0; i<NUM_FP_BYTES; i++) { fast_presses[i]++; }
}

void reset_fast_presses() {
	uint8_t i;
	for(i=0; i<NUM_FP_BYTES; i++) { fast_presses[i] = 0; }
}

int main(void)
{

	DDRB |= (1 << PWM_PIN);	 // Set PWM pin to output, enable main channel
	TCCR0A = FAST; // Set timer to do PWM for correct output pin and set prescaler timing
	TCCR0B = 0x01; // Set timer to do PWM for correct output pin and set prescaler timing


    // Does not necessarily have to be used now because we have not implemented saving to memory at all so all is defaultly on 0
	reset_state(); // Read config values and saved state
	count_modes(); // Enable the current mode group

	// check button press time, unless we're in group selection mode
	if (actual_level_id != GROUP_SELECT_MODE) {
		if ( we_did_a_fast_press() ) { // sram hasn't decayed yet, must have been a short press
			increment_fast_presses();
			next_mode(); // Will handle wrap arounds
		} else { // Long press, keep the same mode
			reset_fast_presses();
			if( !memory_is_enabled() ) {actual_level_id = 0;}  // if memory is turned off, set actual_level_id to 0
		}
	}
	save_mode();

	ADC_on();

    //TURBO ramp down
	uint8_t i = 0;
	uint16_t ticks = 0;
	uint8_t adj_output = 255;

    // VOLTAGE_MON
	uint8_t lowbatt_cnt = 0;
	uint8_t voltage;

	if(actual_level_id > num_available_levels) { output = actual_level_id; }  // special modes, override output
	else { output = available_levels[actual_level_id]; }
	
	while(1) {
		if (fast_presses[0] >= 12) {  // Config mode if 12 or more fast presses
			_delay_s();	   // wait for user to stop fast-pressing button
			reset_fast_presses(); // exit this mode after one use

			toggle_mode(GROUP_SELECT_MODE, 1); // Enter the mode group selection mode?
			toggle_options((config ^ 0b00010000), 2); // memory
			toggle_options((config ^ 0b00100000), 3); // hidden blinkies
			toggle_options((config ^ 0b01000000), 4); // hidden battcheck
			toggle_options((config ^ 0b10000000), 5); // turbo timer
			toggle_options(DEFAULTS_CONFIG, 6); // reset to defaults
		}
		else if (output == STROBE) { // 10Hz tactical strobe
			for(i=0;i<8;i++) {
				set_level(PWM_RAMP_SIZE);
				_delay_ms(33);
				set_output(0);
				_delay_ms(67);
			}
		}
		else if (output == BEACON) {
			set_level(PWM_RAMP_SIZE);
			_delay_ms(10);
			set_output(0);
			_delay_s(); _delay_s();
		}
		else if (output == BATT_CHECK) {
			 blink(battcheck(), CONFIG_BLINK_SPEED/4);
			 _delay_s(); _delay_s();
		}
		else if (output == GROUP_SELECT_MODE) {
			actual_level_id = 0; // exit this mode after one use

			for(i=0; i<NUM_LEVEL_GROUPS; i++) {
				toggle_options(((config & 0b11110000) | i), i+1);
			}
			_delay_s();
		}
		else {
			if ((output == ID_TURBO) && ( ttimer_is_enabled() ) && (ticks > (TURBO_MINUTES * TICKS_PER_MINUTE))) {
				if (adj_output > TURBO_LOWER) { adj_output = adj_output - 2; }
				set_output(adj_output);
			}
			else {
				ticks ++; // count ticks for turbo timer
				set_level(output);
			}

			_delay_ms(500);  // Otherwise, just sleep.

		}
		reset_fast_presses();

		if (ADCSRA & (1 << ADIF)) {  // if a voltage reading is ready
			voltage = ADCH;  // get the waiting value
	
			if (voltage < ADC_LOW) { // See if voltage is lower than what we were looking for
				lowbatt_cnt ++;
			} else {
				lowbatt_cnt = 0;
			}
			
			if (lowbatt_cnt >= 8) {  // See if it's been low for a while, and maybe step down
				//set_output(0);  _delay_ms(100); // blink on step-down:

				if (output > PWM_RAMP_SIZE) {  // blinky modes
					output = PWM_RAMP_SIZE / 2; // step down from blinky modes to medium
				} else if (output > 1) {  // regular solid mode
					output = output - 1; // step down from solid modes somewhat gradually
				} else { // Already at the lowest mode
					set_output(0); // Turn off the light
					set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down as many components as possible
					sleep_mode();
				}
				set_level(output);
				lowbatt_cnt = 0;
				_delay_s(); // Wait before lowering the level again
			}

			ADCSRA |= (1 << ADSC); // Make sure conversion is running for next time through
		}

	}
}

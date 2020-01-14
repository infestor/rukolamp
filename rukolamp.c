/*
 */
// Choose your MCU here, or in the build script
#define ATTINY 13
#define NANJG_LAYOUT
#define __AVR_ATtiny13A__

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <string.h>

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
#define CONFIG_EEPROM_ADDRESS EEPSIZE -1

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

#define FINE_RAMP_SIZE 16
#define FINE_RAMP_VALUES 5, 15, 26, 39, 55, 74, 91, 104, 120, 134, 145, 157, 172, 197, 225, 255 //by 1/16th of 100%
#define RAMPING_TRIGGER_VALUE_DOWN -1
#define RAMPING_TRIGGER_VALUE_UP 1

// These need to be in sequential order, and numbered from 0, no gaps.
// Make sure to update FIRST_BLINKY and LAST_BLINKY as needed.
#define FIRST_BLINKY 0
#define BLINKY_BATT_CHECK 0
#define BLINKY_STROBE	1
#define BLINKY_BEACON 2
#define LAST_BLINKY 2

#define MODE_NORMAL 0
#define MODE_BLINKY 1
#define MODE_RAMPING 2
#define MODE_BIKE 3
#define LAST_NORMAL_MODE_ID MODE_BIKE

#define CONFIGURATION_MODE_ID LAST_NORMAL_MODE_ID + 1
#define GROUP_SELECT_MODE CONFIGURATION_MODE_ID + 1

#define CONFIG_BLINK_BRIGHTNESS	15 // output to use for blinks on battery check (and other modes) = 10%
#define CONFIG_BLINK_SPEED		 750 // ms per normal-speed blink

#define TURBO_MINUTES 1 // when turbo timer is enabled, how long before stepping down
#define TICKS_PER_MINUTE 30 // used for Turbo Timer timing
#define TURBO_LOWER 128  // the PWM level to use when stepping down
#define ID_TURBO PWM_RAMP_SIZE	// Convenience code for turbo mode (id of 100% mode in pwm ramp)


// Calibrate voltage and OTC in this file:
#include "tk-calibration.h"

/*
 * =========================================================================
 */

// Ignore a spurious warning, we did the cast on purpose
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"

#define OWN_DELAY		   // Don't use stock delay functions.
//#define USE_DELAY_MS	 // use _delay_ms()
//#define USE_DELAY_S		 // Also use _delay_s(), not just _delay_ms()
#define USE_DELAY_4MS  //Suitable for all situations we need to handle and uses only uint8_t as input so it saves few bytes of rom
#include "tk-delay.h"

#define NUM_FP_BYTES 3
uint8_t fast_presses[NUM_FP_BYTES] __attribute__ ((section (".noinit")));

// Blinky modes, first and last entry must correspond with FIRST_BLINKY and LAST_BLINKY
//const uint8_t blinky_mode_list[] PROGMEM = { BLINKY_BATT_CHECK, BLINKY_STROBE, BLINKY_BEACON };

// Modes (gets set when the light starts up based on saved config values)
const uint8_t pwm_ramp_values[] PROGMEM = { PWM_RAMP_VALUES };

const uint8_t pwm_fine_ramp_values[] PROGMEM = { FINE_RAMP_VALUES };

#define NUM_LEVEL_GROUPS 8 // Can define up to 16 groups, theoretically the group can have up to 16 level entries
const uint8_t level_groups[] PROGMEM = {
	1, 2, 4, 5, 7, 8, 0,
	3, 5, 7, 8, 0,
	4, 6, 8, 0,
	1, 0,
	2, 0,
	5, 0,
	1, 2, 3, 0,
	3, 7, 0,
};
// has to be the same lenght as the longest level group (in our case 6)
#define LONGEST_LEVEL_GROUP 6 // dont forget to update if editing groups
uint8_t level_group_values[LONGEST_LEVEL_GROUP] __attribute__ ((section (".noinit")));

register uint8_t actual_level_id asm("r3");
register uint8_t actual_mode asm("r4");
register uint8_t actual_pwm_output asm("r5");
register uint8_t config asm("r6");
register uint8_t status asm("r7");
register int8_t ramping_trigger asm("r8");
register uint8_t power_reduction asm("r9");
register uint8_t eepos asm("r10");

// =========================================================================

//inline uint8_t config_level_group_number() { return (config     ) & 0b00001111; }
//inline uint8_t config_memory_is_enabled()  { return (config >> 4) & 0b00000001; }

inline uint8_t status_mode()     { return (status     ) & 0b00000011; }
inline uint8_t status_level_id() { return (status >> 2) & 0b00001111; }
//inline void set_status_mode(uint8_t new_mode)      { status = (status & 0b11111100) | (new_mode & 0b00000011); }
//inline void set_status_level_id(uint8_t new_level) { status = (status & 0b11000011) | ((new_level & 0b00001111) << 2); }

inline uint8_t WeDidAFastPress() {
	uint8_t i;
	for(i = 0; i < NUM_FP_BYTES-1; i++) { if(fast_presses[i] != fast_presses[i+1]) return 0;}
	return 1;
}

inline void IncrementFastPresses() {
	uint8_t i;
	for(i = 0; i < NUM_FP_BYTES; i++) { fast_presses[i]++; }
}

void ResetFastPresses() {
	uint8_t i;
	for(i = 0; i < NUM_FP_BYTES; i++) { fast_presses[i] = 0; }
}

void SaveStatusAndConfig() {  // save the current mode index (with wear leveling)
	uint8_t new_status = actual_mode | (actual_level_id << 2);

	if (status != new_status) {
		uint8_t oldpos = eepos;
		eepos = (eepos + 1) & ((EEPSIZE / 2) - 1);  // wear leveling, use next cell
		eeprom_write_byte((uint8_t *)(eepos), new_status);  // save current status
		eeprom_write_byte((uint8_t *)(oldpos), 0xff);     // erase old state
	}

	//update config if necessary
	if (eeprom_read_byte((uint8_t *)CONFIG_EEPROM_ADDRESS) != config) {
		eeprom_write_byte((uint8_t *)CONFIG_EEPROM_ADDRESS, config);
	}
}

//EMPTY_INTERRUPT(BADISR_vect); //just for case - eliminated by custom startup files

ISR(WDT_vect, ISR_NAKED)
{
	cli();
	ResetFastPresses();
	//turn off watchog (we already had our 1sec)
	WDTCR = 0;
	SaveStatusAndConfig();
	__asm__("ret\n\t"); //trick how to leave interrupts turned off after returning from function
}

inline void FirstBootState() {
	config = DEFAULTS_CONFIG;
	status = DEFAULTS_STATE;
}

inline void RestoreStatusAndConfig() {
	uint8_t eep;
	uint8_t first = 1;

	config = eeprom_read_byte((const uint8_t *)CONFIG_EEPROM_ADDRESS);

	// find the mode index data
	for(uint8_t i = 0; i < (EEPSIZE / 2); i++) {
		eep = eeprom_read_byte((const uint8_t *)i);
		if (eep != 0xff) {
			eepos = i;
			status = eep;
			first = 0;
			break;
		}
	}
	// if no mode_idx was found, assume this is the first boot
	if (first) {
		FirstBootState();
	}

	actual_mode = status_mode();
	actual_level_id = status_level_id();
}

void SetOutputPwm(uint8_t pwm_value) {
	uint8_t desired_power = pwm_value - power_reduction;
	if (desired_power < 15) { TCCR0A = PHASE; } else { TCCR0A = FAST; }
	PWM_LVL = desired_power;
	actual_pwm_output = pwm_value; //this is right! we need to remember what we want actually. Little bit tricky
}

void SetLevel(uint8_t level_id) {
	uint8_t pwm_value = pgm_read_byte(&pwm_ramp_values[level_group_values[level_id] - 1]);
	SetOutputPwm(pwm_value);
}

void blink(uint8_t val, uint8_t speed)
{
	for (; val > 0; val--)
	{
		SetOutputPwm(CONFIG_BLINK_BRIGHTNESS);
		_delay_4ms(speed);
		SetOutputPwm(0);
		_delay_4ms(speed);
		_delay_4ms(speed);
	}
}

uint8_t ReadAndCountPwmLevelsForGroup(uint8_t target_group)
{
	uint8_t group = 0, level, mc=0;

	for(uint8_t i = 0; i < (sizeof(level_groups) / sizeof(level_groups[0])); i++) {
		level = pgm_read_byte(&level_groups[i]);
		// if we hit a 0, that means we're moving on to the next group
		if (level == 0) {
			group++;
			if (group > target_group) break;
		}
		// else if we're in the right group, store the mode and increase the mode count
		else if (group == target_group) {
			level_group_values[mc] = level;
			mc++;
		}
	}

	return mc;
}

uint8_t CountNumLevelsForGroupAndMode(uint8_t target_mode) {
	uint8_t mc=0;

	if ((target_mode == MODE_NORMAL)) {
		//mc = ReadAndCountPwmLevelsForGroup(config_level_group_number());
		mc = ReadAndCountPwmLevelsForGroup(config); //using just config saves few bytes
	}
	else if (target_mode == MODE_BIKE) { //bike only uses the default 6 modes (group 0)
		ReadAndCountPwmLevelsForGroup(0);
		mc = 5;
	}
	else if (target_mode == MODE_BLINKY) {
		mc = LAST_BLINKY; //for blinky mode - levels are in fact blinkies
	}

	return mc;
}

inline void NextLevel() {
	if (actual_mode != MODE_RAMPING) {
		uint8_t num_available_levels = CountNumLevelsForGroupAndMode(actual_mode);
		actual_level_id++;
		// if we hit the end of list, go to first
		if (actual_level_id == num_available_levels) {
			actual_level_id = 0;
		}
		ramping_trigger = 0;
	}
	else {
		// handle trigger of ramping until next press of switch to stay at that new level
		if (ramping_trigger == 0) {
			ramping_trigger = RAMPING_TRIGGER_VALUE_UP; //start ramping
		}
		else {
			ramping_trigger = 0; //stop at new level
		}
	}
}

inline void NextMode() {
	// go to next mode
	actual_mode++;
	if (actual_mode > LAST_NORMAL_MODE_ID) actual_mode = MODE_NORMAL;
	if (actual_mode == MODE_RAMPING) ramping_trigger = RAMPING_TRIGGER_VALUE_UP;
	//set_status_mode(actual_mode);
	//set_status_level_id(0);
	actual_level_id = 0;
	// Since we start on each mode always on level_id 0, we dont need to know real number of levels here. But just in NextMode() function
	// But we need the levels :(
}

// =========================================================================

int __attribute__((noreturn,OS_main)) main (void)
{

	DDRB |= (1 << PWM_PIN);	 // Set PWM pin to output, enable main channel
	TCCR0A = FAST; // Set timer to do PWM for correct output pin and set prescaler timing
	TCCR0B = 0x01; // Set timer to do PWM for correct output pin and set prescaler timing

	ADC_on();

	//start watchdog to measure one second from start to be able to clear fast presses independetly from main loop where sleeps and other stuff happens
	wdt_reset();
	WDTCR = (1 << WDCE);
	WDTCR = (1 << WDTIE) | WDTO_1S; //1sec timeout
	sei();

	// check button press time, unless we're in group selection mode
	if ( WeDidAFastPress() ) { // sram hasn't decayed yet, must have been a short press
		IncrementFastPresses();

		// triple-tap from a solid mode
		if(fast_presses[0] == 5) {
			NextMode();
		}
		else if (fast_presses[0] >= 10) {  // Config mode if 10 or more fast presses
			// Enter into configuration
			WDTCR = (1 << WDCE);
			WDTCR = (1 << WDTIE) | WDTO_2S; //prolong autosave to 2 sec
			blink(2, 18);
			wdt_reset();
			_delay_4ms(250);	   // wait for user to stop fast-pressing button

			config++;
			if (config > NUM_LEVEL_GROUPS) config = 0;
			_delay_4ms(250);
			ResetFastPresses(); // exit this mode after one use
			blink(config, 50);
		}
		else {
			NextLevel(); //this includes also changing of blinky modes, because they are taken from list the same way as levels
		}
	}
	else { // Long press / power on after long off. Definitely means reset
		ResetFastPresses();

		// Does not necessarily have to be used now because we have not implemented saving to memory at all so all is defaultly on 0 anyway
		RestoreStatusAndConfig(); // Read config values and saved state / or use defaults
	}

	CountNumLevelsForGroupAndMode(actual_mode);

    //TURBO ramp down + undervoltage protection
	power_reduction = 0; //just for sure
	uint8_t lowbatt_cnt = 0; //better here because get reseted after every switch press
	uint8_t ticks = 0;
	uint8_t adj_output = 255;

	for(;;) {
		if (actual_mode == MODE_BLINKY)
		{
			if (actual_level_id == BLINKY_STROBE) {
				for(uint8_t i = 0; i < 7; i++) {
					SetOutputPwm(255);
					_delay_4ms(3);
					SetOutputPwm(0);
					actual_pwm_output = 255; //little hack for un-confuse low voltage protection mechanism
					_delay_4ms(75);
				}
			}
			else if (actual_level_id == BLINKY_BEACON) {
				SetOutputPwm(255);
				_delay_4ms(3);
				SetOutputPwm(0);
				actual_pwm_output = 255; //little hack for un-confuse low voltage protection mechanism
				_delay_4ms(250);
				_delay_4ms(250);
			}
			else if (actual_level_id == BLINKY_BATT_CHECK) {
				blink(battcheck(), CONFIG_BLINK_SPEED / 16);
				actual_pwm_output = CONFIG_BLINK_BRIGHTNESS; //little hack for un-confuse low voltage protection mechanism
				_delay_4ms(250);
				_delay_4ms(250);
			}
		}
		else if (actual_mode == MODE_RAMPING) {
			if (ramping_trigger != 0) {
				actual_level_id += ramping_trigger;
				if (actual_level_id == (FINE_RAMP_SIZE - 1)) ramping_trigger = RAMPING_TRIGGER_VALUE_DOWN; //handles top end
				if (actual_level_id == 0) { ramping_trigger = RAMPING_TRIGGER_VALUE_UP;} //handles low end
			}
			SetOutputPwm(pgm_read_byte(&pwm_fine_ramp_values[actual_level_id]));
			if (ramping_trigger != 0) {
				_delay_4ms(31);
			} else {
				_delay_4ms(250);
				_delay_4ms(250);
			}
		}
		else if (actual_mode == MODE_NORMAL)
		{
			if (level_group_values[actual_level_id] == ID_TURBO) {
				if (ticks > (TURBO_MINUTES * TICKS_PER_MINUTE)) {
					if (adj_output > TURBO_LOWER) { adj_output = adj_output - 2; }
					SetOutputPwm(adj_output);
				}
				else {
					SetLevel(actual_level_id);
					ticks++; // count ticks for turbo timer
				}
			}
			else {
				SetLevel(actual_level_id);
			}

			_delay_4ms(250);
			_delay_4ms(250);
		}
		else // Definitely has to be MODE_BIKE
		{
				SetLevel(actual_level_id);
				_delay_4ms(250);
				_delay_4ms(250);
				SetOutputPwm(255);
				_delay_4ms(3);
				SetLevel(actual_level_id);
				_delay_4ms(37);
				SetOutputPwm(255);
				_delay_4ms(3);
				SetLevel(actual_level_id);
		}

		// INFO: Need to keep all modes branch ifs (beacon, etc) to run approx 2sec, to properly work undervoltage reduction cycle speed

		//ResetFastPresses(); // Probably already cleared by interrupt from watchdog, i think I will remove it from this location

		// Battery undervoltage protection
		if (ADCSRA & (1 << ADIF)) {  // if a voltage reading is ready
			uint8_t voltage = ADCH;  // get the waiting value

			if ((voltage < ADC_LOW) && (ramping_trigger == 0)) { // See if voltage is lower than what we were looking for
				lowbatt_cnt++;
			} else {
				lowbatt_cnt = 0;
			}

			if (lowbatt_cnt >= 2) {  // See if it's been low for a while, and maybe step down
				//somehow scale the step according to expected level
				uint8_t decrease_step;
				if (actual_pwm_output > 200) { decrease_step = 10; }
				else if (actual_pwm_output > 150) { decrease_step = 7; }
				else if (actual_pwm_output > 100) { decrease_step = 5; }
				else if (actual_pwm_output > 50) { decrease_step = 3; }
				else { decrease_step = 1; }

				power_reduction += decrease_step;

				if (power_reduction > actual_pwm_output) { // Already at the lowest mode
					PWM_LVL = 0; //SetOutputPwm(0); // Turn off the light
					set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down as many components as possible
					sleep_mode();
				}
				//SetOutputPwm(0); cannot be used because it effectively sets variable actual_pwm_output to 0 therefore we cannot revert back original level
				//TCCR0A = PHASE;
				PWM_LVL = 0; _delay_4ms(1); // blink on step-down
				//SetOutputPwm(actual_pwm_output); //refresh ouput using new power reduction not needed to refresh here, because it will do itself next round somewhere
				lowbatt_cnt = 0;
				//_delay_s(); // Wait before lowering the level again
			}

			ADCSRA |= (1 << ADSC); // Make sure conversion is running for next time through
		}

	}
}

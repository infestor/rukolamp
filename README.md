## Rukolamp

LED Flashlight 1-channel driver custom firmware for Nanjg105c and simmilar based on AtTiny13A controller.<br>
Based on ghart's babka firmware, which was based on ToyKeeper biscotti firmware ([https://launchpad.net/flashlight-firmware](https://launchpad.net/flashlight-firmware)).

* Has 4 separate main modes/mode-groups: Normal, Blinkies, Ramping and Bike.
* Cycle through Modegroups is done by 5x fast-click 
* Battery undervoltage protection for all modes - when voltage < 3.0V is detected, intensity is lowered every 2 seconds by small step (relative to intended output) followed by very short 5ms blink. Repeats until battery voltage rises above 3V. When there is no room to lower more, power down mode is initiated. (maybe it could use mode smart logic, but there was no room left in processor flash :(
* Turbo ramp-down function - when turbo (255) level is selected in normal mode, after 1 minute it starts slowly ramping down for another minute to 50% of power.
* 8 selectable level-groups
* Last mode/level memory - eeprom write is initiated after 1 second of idle (wear leveling of eeprom - 32bytes cyclic use, should cover at least 1.5million last-state writes)

_I would implement more stuff or some functions smarter, but unfortunately I got out of available flash (512 instructions/words or 1024B) even when I used all options to optimize size known to me_

#### Processor pins used:
* PB01: as PWM output
* PB02: ADC measuring with voltage divider (30kOhm : 10kOhm), so for example 4.2V is effectively 1.05V at the processor and against 1.1V internal reference should provide result 244 (when left adjusted).

Fuses for processor are Lo: 0x75, Hi: 0xFF.<br>
That gives cpu frequency of 4.8MHz and PWM frequency 18.7kHz (for super low pwm levels there is used phase-correct pwm mode with 9.4kHz frequency).

---

### Modes:
Switchable by 5times fast click

**(1)Mode - normal:**

Classic mode.
Intensity switchable by one fast click, from last intensity rewinds on 1.st
Values predefined in selected level-group.
Default level group 1 - (1%, 10%, 33%, 50%, 75%, 100%)

**(2)Mode - blinkies:**
Has 3 sub-modes. Cycle through them by one fast click.

1. Battcheck (10% intensity blinks), number of blinks (0-9) tells actual percentage of remaining battery level, then 2sec pause and blinks again. Infinite loop
2. Strobe (100% intensity, 10ms ON / 300ms OFF)
3. Beacon (100% intensity, 10ms once in 2s)

**(3)Ramping mode:**

Has 16 predefined fine intensity steps.
When this mode is selected, ramping by 1 step every 125ms is initiated. Starting from lowest level, going up. When top is reached, goes down in reverse order to bottom and so on.
Ramping is stopped on actual level by fast-click. Another fast-click starts the ramping again - always with direction from actual level up. 

**(4)Bike mode:**

Simmilar to normal mode, but uses always only the default level group 1 without last (100%) intensity so - 1%, 10%, 33%, 50%, 75%.<br>
Every 2 seconds there is intensity glitch like this: 100% 15ms, original level 150ms, 100% 15ms, then original level for another 2sec and glitch again. 

**(x)Configuration:**
_It's a little bit tricky, because this was implemented last and lack of flash forced me to make it quite not so user friendly._

10 or more fast clicks activates this mode

* Signalisation that mode is active - 8 fast blinks (On=40ms/off=80ms).
* From this moment do not press anything
* Then there is 0.8sec protection time to stop pressing button.
* Level group changes - next level group up, or when on last group go to first
* Visual confirmation of new group by number of blinks (approx 2 blinks/sec)
* Now there is another 1.25s protective interval and two possibilities:
* *1. Leave it and do not press anything - the level group will be used and saved in config, config mode will be exited and you will return to previous mode.
* *2. Fast click again, which will leave us in configuration mode and restarts the whole process from signalisation further (but with remembering that previously the level-group was increased, which could help speed up changes by more than one group at a time.

---

**Level groups:**<br>
1] 1%, 10%, 33%, 50%, 75%, 100% __(default)__<br>
2] 25%, 50%, 75%, 100%<br>
3] 33%, 66%, 100%<br>
4] 1% (when borrowed to children etc.)<br>
5] 10%<br>
6] 50%<br>
7] 1%, 10%, 25% (for camping etc.)<br>
8] 25%, 75%<br>

**Intensity levels vs raw pwm values for level-groups:**<br>
PWM ramp size 8

| 1 | 2  | 3  | 4  | 5  | 6  | 7  | 8   | # |
|---|---|---|---|---|---|---|---|---|
| 1 | 10 | 25 | 33 | 50 | 66 | 75 | 100 | [%] |
| 5 | 26 | 64 | 85 | 128| 169| 192| 255 | [pwm] |

**Raw pwm values for Ramping mode:**

|5|15|26|39|55|74|91|104|120|134|145|157|172|197|225|255|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|1%|||||||||||||||100%|

**Battcheck: blinks vs voltage:**

|0|1|2|3|4|5|6|7|8|9|#blinks|
|---|---|---|---|---|---|---|---|---|---|---|
|3.0 >|3.0|3.3|3.5|3.7|3.8|3.9|4.0|4.1|4.2|[V]|

**Undervoltage power reduction steps:**

|Intended raw pwm level|0 - 50|51-100|101-150|151-200|201<|
|---|---|---|---|---|---|
|reduced by this every 2s|-1|-3|-5|-7|-10|


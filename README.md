## Rukolamp

LED Flashlight driver firmware
_(for Nanjg105c and simmilar with AtTiny13A controller)_

### Mody:

**(1)normal:**

default 6 intenzit (1%, 10%, 33%, 50%, 75%, 100%)

**(2)special:**

- battcheck (10%)
- strobe (100% 100ms=1/200ms=0 - 3x za vteřinu)
- beacon (100%/50ms jednou za 2s)

**(3)Ramping mode:**

TBD

**(4)bike mód:**

jede na přepínání intenzit jako v normal módu akorát bez 100%, protože to by nedávalo smysl
(nejspíš jen těch 6 default, ať byla grupa jakákoliv) 

**(x)Configuration:**
1. level mode group
2. memory toggle

---

**Other features:**
- přepínání módů TROJKLIK
- low voltage protection
- turbo ramp down
- skok do konfigurace 10x klik

---

**level módy:**<br>
1] 1%, 10%, 33%, 50%, 75%, 100%<br>
2] 25%, 50%, 75%, 100%<br>
3] 33%, 66%, 100%<br>
4] 1% (pro děti na hraní)<br>
5] 10%<br>
6] 50%<br>
7] 1%, 10%, 25% (třeba do stanu)<br>
8] 25%, 75%<br>

**všechny použité intenzity:**<br>
PWM ramp size 8

| 1 | 2  | 3  | 4  | 5  | 6  | 7  | 8   | # |
|---|---|---|---|---|---|---|---|---|
| 1 | 10 | 25 | 33 | 50 | 66 | 75 | 100 | [%] |
| 5 | 26 | 64 | 85 | 128| 169| 192| 255 | [pwm] |

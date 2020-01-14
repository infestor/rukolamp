/* $Id$ */

#if (__GNUC__ < 3) || (__GNUC__ == 3 && __GNUC_MINOR__ < 3)
#error "GCC version >= 3.3 required"
#endif

#include "attiny13a.S"

#include "macros.inc"

    .section .vectors,"ax",@progbits
    .global __vectors
    .func   __vectors
__vectors:
    RJMP   __init
    RETI
    RETI
    RETI
    RETI
    RETI
    RETI
    RETI

    .weak __vector_8
    RJMP __vector_8

    RETI
    RETI
    RETI
    RETI
    RETI
    RETI
    .endfunc

    .text
    .section .init0,"ax",@progbits
    .weak   __init
;   .func   __init
__init:

#ifndef __AVR_ASM_ONLY__
    .weak   __stack
    .weak   __heap_end
    .set    __heap_end, 0

    .section .init2,"ax",@progbits
    clr __zero_reg__
    out AVR_STATUS_ADDR, __zero_reg__
    ldi r28,lo8(__stack)
    out AVR_STACK_POINTER_LO_ADDR, r28

    .set    __stack, RAMEND
#endif /* !__AVR_ASM_ONLY__ */


    .section .init9,"ax",@progbits
    RJMP   main

    .section .note.gnu.avr.deviceinfo, "", @note
#define NOTE_NAME "AVR"
#ifdef __AVR_DEVICE_NAME__
    #define DEVICE_NAME STR(__AVR_DEVICE_NAME__)
#else
    #define DEVICE_NAME ""
#endif

    .long .L__note_name_end - .L__note_name_start
    .long .L__desc_end - .L__desc_start
    .long 1 ; Type 1 - this is the only known note type for AVR.
.L__note_name_start:
    .asciz NOTE_NAME
    .balign 4
.L__note_name_end:
.L__desc_start:
#ifdef FLASHSTART
    .long FLASHSTART
#else
    .long 0
#endif
#ifdef PROGMEM_SIZE
    .long PROGMEM_SIZE
#elif FLASHEND > 0
    .long FLASHEND + 1
#else
    .long FLASHEND
#endif
    .long RAMSTART
#ifdef RAMSIZE
    .long RAMSIZE
#elif RAMEND > 0
    .long RAMEND - RAMSTART + 1
#else
    .long RAMEND
#endif
#ifdef E2START
    .long E2START
#else
    .long 0
#endif
#ifdef EEPROM_SIZE
    .long EEPROM_SIZE
#elif E2END > 0
    .long E2END + 1
#else
    .long E2END
#endif
    /* String offsets table.
    Index 0 - Size of offset table in bytes
    Index 1 - Device name byte offset
    */
.L__stroffsettab_start:
.long .L__stroffsettab_end - .L__stroffsettab_start /* Size of index table in bytes */
.long .L__device_name_start - .L__strtab_start /* Byte offset of device name */
.L__stroffsettab_end:
    /* String table for storing arbitrary strings.
    Offsets are stored in the string offset table above */
.L__strtab_start:
    .byte 0
.L__device_name_start:
    .asciz DEVICE_NAME
.L__device_name_end:
    .byte 0
.L__strtab_end:
.L__desc_end:
    .balign 4

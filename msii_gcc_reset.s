;****************************************************
;This program required because stupid CW won't allow
;jmp to an address - says "nothing more expected"
;and won't go any further.
;****************************************************
.sect .text1
.globl reboot
.globl monitor
.equ CORE1,       0x00           ;ports A, B, E, modes, inits, test
.equ INITRM,      CORE1+0x10     ;initialization of internal RAM position register
.equ INITRG,      CORE1+0x11     ;initialization of internal registers position register
.equ INITEE,      CORE1+0x12     ;initialization of internal EEPROM registers position register

reboot:
  jmp 0xF800						 ;go to serial mon pgm & restart ms II code
  rts
monitor:
  movb  #0x00,INITRG    ;set registers at $0000 
  movb  #0x39,INITRM    ;set ram to end at $3fff 
  movb  #0x09,INITEE    ;set eeprom to end at $0fff
  clra                  ; clear A reg
  sei;                  ; disable interrupts 
  jmp 0xF842            ;go to serial mon pgm & wait for new code reload
  rts

.sect .text
.globl SpSub
.globl NoOp
;*******************************************************************
;* MOTOROLA
;*
;* DESCRIPTION: S12 single array Flash routines
;* SOURCE: flash.c
;* COPYRIGHT: © 04/2004 Made in the USA
;* AUTHOR: rat579
;* REV. HISTORY: 060304 - fixed CCR return value and optimized
; in SpSub routine
;*
;*******************************************************************/
;*****************************************************************************
; Local defines
;*****************************************************************************
.equ FSTAT, 0x105
.equ CCIF,  0x40
;*********************************************************************
;* SpSub - register flash command and wait for Flash CCIF
;* this subroutine is copied into ram before executing
;* because you can't execute out of flash while a flash command is
;* in progress.
;*
;* Note: must be even # of bytes!
;*
;* Uses 32 bytes on stack + 2 bytes for JSR
;*********************************************************************
NoOp:
  nop
.align 2                   ;Make code start word aligned
SpSub:
  tfr ccr,b                ;get copy of ccr
  orcc #0x10               ;disable interrupts
  staa FSTAT               ;[PwO] register command
  nop ;[O] wait min 4~ from w cycle to r
  nop ;[O]
  nop ;[O]
  brclr FSTAT,CCIF,. ;[rfPPP] wait for queued commands to finish
  tfr b,ccr                ;restore ccr and int condition
  rts                      ;back into DoOnStack in flash
SpSubEnd:

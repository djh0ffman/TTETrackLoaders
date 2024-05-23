
;-----------------------------------------------------------
;
; TTE Turbo Track
;
; A fast byte loader with integrated ZX0 unpacker
; It uses 2xMFM buffers so that unpacking can occur while
; the disk DMA is active. It also uses the 
; 
;
; based on code Written by Frank Wille
;
; USAGE
;
; call TTE_Turbo_Read with the following parameters
;
; d0 = start byte
; d1 = length in bytes
; d2 = drive number ( not implemented )
; d3 = 0 = load / 1 = zx0 unpack 
; a0 = destination
; a1 = mfm buffer ( size is 2 x MFM_BUFSIZE )
;
;-----------------------------------------------------------
       incdir     include
       incdir     include/hardware/
       include    hw.i
       include    intbits.i
       include    cia.i
       include    "tteturbo.i"


; Port A bits (input)
DSKCHANGE    equ 2
DSKPROT      equ 3
DSKTRACK0    equ 4
DSKRDY       equ 5

; Port B bits (output)
DSKSTEP      equ 0
DSKDIREC     equ 1
DSKSIDE      equ 2
DSKSEL0      equ 3
DSKMOTOR     equ 7

; constants
NUM_TRACKS   equ 160
SECT_PER_TRK equ 11
STEPDELAY    equ 64                                                     ; (* 64us) ca. 4ms  (minimum: 3ms)
SETTLEDELAY  equ 400                                                    ; (* 64us) ca. 25ms (minimum: 18ms)

MFM_BUFSIZE  equ (MFM_READLEN+1)*2
MFM_READLEN  equ $18d0                                                  ; in words
;MFM_WRITELEN equ $1900                                                  ; in words
;MFM_GAPSIZE  equ $f0                                                    ; gap in words at track-start and end

NUM_RETRIES  equ 4                                                      ; retries on read error
DMA_TIMEOUT  equ 2000                                                   ; timeout after 2 seconds

CIAA_CIAPRA    = $BFE001
CIAB_CIAPRB    = $BFD100
                 

; d0 = start byte
; d1 = length in bytes
; d2 = drive number ( not implemented )
; d3 = 0 = load / 1 = zx0 unpack 
; a0 = destination
; a1 = dual mfm buffers

TTE_TurboRead:
       movem.l    d1-a6,-(sp)

       lea        CIAA_CIAPRA,a2
       lea        CIAB_CIAPRB,a3
       lea        TTETrackVars(pc),a5
       lea        CUSTOM,a6

       clr.w      TTETrack_Done(a5)

       ; disable current disk DMA transfer, but enable disk DMA in general
       move.w     #$4000,DSKLEN(a6)
       move.w     #$8210,DMACON(a6)

	; disable DSKSYN & DSKBLK
       move.w     #$1002,INTENA(a6)

       move.l     a0,TTETrack_Dest(a5)
       move.l     a1,TTETrack_MFM1(a5)
       lea        MFM_BUFSIZE(a1),a1
       move.l     a1,TTETrack_MFM2(a5)

       move.l     d1,TTETrack_Length(a5)
       move.w     d3,TTETrack_Mode(a5)

       move.l     d0,d6
       divu       #$200,d0
       swap       d0
       clr.w      d0
       swap       d0
       divu       #SECT_PER_TRK,d0
       move.w     d0,d5                                                 ; start track
       swap       d0
       move.w     d0,TTETrack_StartSector(a5)                           ; start sector

       and.w      #$1ff,d6
       move.w     d6,TTETrack_StartByte(a5)                             ; byte offset into sector

       bsr        td_motoron                                            ; select the drive, start motor

       move.w     TTETrack_CurrentTrack(a5),d4
       bmi        .hardseek

       ; now snoop the track number to make sure its right
       move.w     #$8010,d0
       bsr        td_startdma
       bsr        td_wait_disk_dma
       beq        .error

       ; decode header
       cmp.w      #$4489,(a4)+
       bne        .error
       move.l     #$55555555,d2
       move.l     (a4)+,d0
       move.l     (a4)+,d1
       and.l      d2,d0
       and.l      d2,d1
       add.l      d0,d0
       or.l       d1,d0                                                 ; header info
       swap       d0
       cmp.b      d0,d4
       bne        .hardseek                                             ; track number doesn't match, re-seek

       move.w     d5,d0
       bsr        td_seek
       bra        .startloading

.hardseek
       bsr        td_seekzero
       move.w     d5,d0
       bsr        td_seek


.startloading
       ; init unpacker vars
       lea        zx0_decompress_tte(pc),a4
       move.l     a4,TTETrack_UnpackState+(5*4)(a5)

       lea        td_dmatick(pc),a4
       move.l     a4,$64.w                                              ; enable DSKBLK interrupts
       move.w     #$8002,INTENA(a6)

       move.w     #$8000|MFM_READLEN+1,d0
       bsr        td_startdma

.stop
       tst.w      TTETrack_Done(a5)
       beq        .stop
       ;move.w     #$1002,INTENA(a6)

       ;bra        .stop
       moveq      #0,d0
.error       
       movem.l    (sp)+,d1-a6
       rts


stopme:
       nop
       rts

;------------------------------------------
; 
; dma tick / interrupt
;
;------------------------------------------

td_dmatick:
       movem.l    d0-a6,-(sp)
       lea        CUSTOM,a6
       move.w     INTREQR(a6),d0
       and.w      #$0002,d0
       beq        .notdisk
       move.w     d0,INTREQ(a6)
       move.w     d0,INTREQ(a6)

       ;move.w     #$00f,COLOR00(a6)

       lea        CIAA_CIAPRA,a2
       lea        CIAB_CIAPRB,a3
       lea        TTETrackVars(pc),a5

       move.w     TTETrack_CurrentTrack(a5),TTETrack_DecodeTrack(a5)

       moveq      #0,d1
       move.w     #$1600,d1
       move.w     TTETrack_StartSector(a5),d0
       mulu       #$200,d0
       add.w      TTETrack_StartByte(a5),d0
       sub.w      d0,d1                                                 ; size of data available in this track

       cmp.l      TTETrack_Length(a5),d1
       bcc        .decodeonly

       move.w     TTETrack_CurrentTrack(a5),d0
       addq.w     #1,d0
       bsr        td_seek

       ; swap the buffers
       move.l     TTETrack_MFM1(a5),a0
       move.l     TTETrack_MFM2(a5),TTETrack_MFM1(a5)
       move.l     a0,TTETrack_MFM2(a5)

       move.w     #$8000|MFM_READLEN+1,d0
       bsr        td_startdma

       bsr        td_sectors
       bne        .error
       bsr        td_decode

.notdisk
       ;move.w     #$000,COLOR00(a6)
       movem.l    (sp)+,d0-a6
       rte

.error
       move.w     #$f00,COLOR00(a6)
       bra        .error

.decodeonly
       bsr        td_motoroff
       move.l     TTETrack_MFM1(a5),a0
       bsr        td_sectors
       bsr        td_decode
       move.w     #$1002,INTENA(a6)                                     ; disable the disk interrupt
       bra        .notdisk


; d0 = DSKLEN

td_startdma:
       ;move.l     MFMbuffer(pc),a0
       move.l     TTETrack_MFM1(a5),a4
       addq.l     #2,a4                                                 ; make room to restore missed SYNC
       move.w     #$4000,DSKLEN(a6)

	;-----------------------------------------------------
	; Disk read DMA fetches a whole track into the buffer
	;-----------------------------------------------------

	; select MFM encoding and enable synchronization word (DSKSYNC)
       ;move.w     #$7200,ADKCON(a6)
       ;move.w     #$8500,ADKCON(a6)

       move.w     #$6600,ADKCON(a6)
       move.w     #$9500,ADKCON(a6)
       move.w     #$4489,DSKSYNC(a6)

	; start disk-read DMA
       move.l     a4,DSKPT(a6)
       move.w     d0,DSKLEN(a6)
       move.w     d0,DSKLEN(a6)
       rts


; ------------------------------------------------
; a0 = mfm buffer
td_decode:
       move.l     #$55555555,d3                                         ; d3: MFM mask
       lea        TTETrack_Sectors(a5),a2
       move.w     TTETrack_StartSector(a5),d0
       add.w      d0,d0
       add.w      d0,d0
       lea        (a2,d0.w),a2

       moveq      #0,d4
       move.w     TTETrack_StartByte(a5),d4

       move.l     TTETrack_Dest(a5),a3

       move.w     #SECT_PER_TRK-1,d7
       sub.w      TTETrack_StartSector(a5),d7
.nextblock
       move.l     (a2)+,a0
       ;move.l     (a1,d5.w),a0                           ; sector data start in MFM buffer

	; decode the block while calculating the checksum
       moveq      #0,d5
       move.l     a0,a4
       lea        512(a0),a1
       moveq      #127,d2
.decode_loop:
       ;move.l     (a0)+,d0
       move.l     (a0),d0
       
       eor.l      d0,d5
       and.l      d3,d0
       move.l     (a1)+,d1
       eor.l      d1,d5
       add.l      d0,d0
       and.l      d3,d1
       or.l       d1,d0
       move.l     d0,(a0)+
       dbf        d2,.decode_loop

	; decode data block checksum and compare with our calculated one
       movem.l    -520(a0),d0/d1
       and.l      d3,d0
       and.l      d3,d1
       add.l      d0,d0
       or.l       d1,d0
       and.l      d3,d5
       cmp.l      d0,d5
       bne        .fuckit                                               ; Checksum error.

       ;add.w      TTETrack_StartByte(a5),a4
       add.l      d4,a4
       moveq      #0,d6
       move.w     #$200,d6
       ;sub.w      TTETrack_StartByte(a5),d6
       sub.l      d4,d6
       cmp.l      TTETrack_Length(a5),d6
       bcs        .notend
       move.l     TTETrack_Length(a5),d6
.notend
       sub.l      d6,TTETrack_Length(a5)
       moveq      #0,d4

       tst.w      TTETrack_Mode(a5)
       beq        .loadonly

       ; -- unpack.. here we go
       ; a4 = source data
       ; a3 = destination
       
       move.l     a3,a1
       move.l     a4,a0
       movem.l    a2/a3,-(sp)
       movem.l    TTETrack_UnpackState(a5),d0/d1/d2/d5/a2/a3
       jsr        (a3)
       movem.l    d0/d1/d2/d5/a2/a3,TTETrack_UnpackState(a5)
       movem.l    (sp)+,a2/a3

;       block check
;       movem.l    d0-a6,-(sp)
;       move.l     a1,d7
;       sub.l      a3,d7
;       subq.w     #1,d7
;
;       moveq      #0,d6
;       move.l     a3,a2
;       sub.l      #MemBuffer,a2
;       add.l      #TestFile,a2
;.check       
;       cmp.b      (a2)+,(a3)+
;       beq        .match
;       addq.w     #1,d6
;.match
;       dbra       d7,.check
;       nop
;       movem.l    (sp)+,d0-a6
;
       move.l     a1,a3
       bra        .unpackdone

       ; -- load only, byte copy decoded chunk to destination
.loadonly
       subq.w     #1,d6
.copyloop       
       move.b     (a4)+,(a3)+
       dbra       d6,.copyloop

.unpackdone

       tst.l      TTETrack_Length(a5)
       bne        .stilldata
       moveq      #0,d7                                                 ; stop looping sectors
       move.w     #1,TTETrack_Done(a5)
.stilldata       
       dbra       d7,.nextblock

       clr.w      TTETrack_StartByte(a5)
       clr.w      TTETrack_StartSector(a5)
       move.l     a3,TTETrack_Dest(a5)
       rts

.fuckit
       bra        .fuckit


; ------------------------------------------------
; a0 = mfm buffer

td_sectors:
	; restore the first SYNC, which is not written by the DMA
       ;move.w     #$4489,(a0)
       addq.l     #2,a0
       lea        TTETrack_Sectors(a5),a1
       lea        MFM_READLEN*2(a0),a2
       move.l     #$55555555,d2
       moveq      #SECT_PER_TRK-1,d3

       move.w     #$4489,-(a0)

	; find one or two SYNCs in front of a header
.3:    cmp.l      a2,a0
       bhs        .err_no_sync                                          ; no sync found, missing sector
       cmp.w      #$4489,(a0)+
       bne        .3
       cmp.w      #$4489,(a0)+
       beq        .4
       subq.l     #2,a0

	; decode the header's info field
.4:    movem.l    (a0),d0-d1
       and.l      d2,d0
       and.l      d2,d1
       add.l      d0,d0
       or.l       d1,d0

	; verify the track number
       swap       d0
       cmp.b      TTETrack_DecodeTrack+1(a5),d0
       bne        .err_wrongtrk
       swap       d0

	; use sector number in bits 8-15 as table index
       clr.b      d0
       lsr.w      #6,d0
       cmp.b      #SECT_PER_TRK<<2,d0
       bhs        .err_header

       ;tst.l      (a1,d0.w)
       ;beq        .5

	; sector already found in buffer, skip to the next one
       ;lea        56+1024(a0),a0
       ;bra        .3

	; enter sector data pointer into the table
.5:    lea        56(a0),a0
       move.l     a0,(a1,d0.w)

	; skip data, scan for next sector
       lea        1024(a0),a0
       dbf        d3,.3

       moveq      #0,d0                                                 ; ok, no error
       bra        .done
.err_empty:
       moveq      #1,d0                                                 ; timeout while waiting for disk-DMA
       bra        .done
.err_no_sync:
       moveq      #2,d0                                                 ; sector not found in MFM buffer
       bra        .done
.err_header:
       moveq      #3,d0                                                 ; illegal sector number in the header
       bra        .done
.err_wrongtrk:
       moveq      #4,d0                                                 ; wrong track number in the header
.done:
       tst.w      d0
       rts

;---------------------------------------------------------------------------
td_wait_disk_dma:
; Wait until DMA transfer is finished. Reset DSKLEN.
; -> d0/N = timeout
; a0 and a1 are preserved, d2 is trashed!

       ;move.w     #$0002,INTREQ(a6)
       move.w     #DMA_TIMEOUT,d2

.1:    moveq      #16,d0
       bsr        td_delay                                              ; ~1ms
       moveq      #2,d0
       and.w      INTREQR(a6),d0
       dbne       d2,.1

       move.w     #$4000,DSKLEN(a6)
       move.w     #$0002,INTREQ(a6)

       tst.w      d2
       rts



;---------------------------------------------------------------------------
td_motoron:
; Turn the drive's motor on and wait until rotating at full speed.
; The drive stays selected.
; a2 = CIAAPRA
; a3 = CIABPRB
       move.b     (a3),d1
       and.b      #1<<DSKSIDE,d1                                        ; save current disk side
       or.b       #$79,d1                                               ; $7d
       move.b     d1,(a3)
       bclr       #3,d1
       move.b     d1,(a3)

       moveq      #100-1,d2                                             ; timeout after ~500ms
.waitdrive:  
       moveq      #78,d0
       bsr        td_delay                                              ; ~5ms
       btst       #DSKRDY,(a2)
       dbeq       d2,.waitdrive
       rts



;---------------------------------------------------------------------------
td_motoroff:
; Turn the motor off for all drives. Deselect all drives.
; a3 = CIABPRB

       or.b       #$f8,(a3)
       nop
       and.b      #$87,(a3)
       nop
       or.b       #$78,(a3)                                             ; deselect all
       rts


;---------------------------------------------------------------------------
td_seek:
; Step to the required cylinder and select the correct head.
; d0.w = track to seek
; a2 = CIAAPRA
; a3 = CIABPRB

       cmp.w      #NUM_TRACKS,d0
       bhs        .exit                                                 ; illegal track

       movem.l    d2-d3,-(sp)
       move.w     TTETrack_CurrentTrack(a5),d3
       move.w     d0,d2
       btst       #0,d2
       bne        .1

	; select lower head
       bset       #DSKSIDE,(a3)
       bclr       #0,d3
       bra        .2

	; select upper head
.1:    bclr       #DSKSIDE,(a3)
       bset       #0,d3

.2:    cmp.w      d3,d2
       beq        .done
       bhi        .3

	; step outwards
       bset       #DSKDIREC,(a3)
       subq.w     #2,d3
       bra        .4

.3:	; step inwards
       bclr       #DSKDIREC,(a3)
       addq.w     #2,d3

.4:    bsr        td_step
       bra        .2

.done:
       move.w     d2,TTETrack_CurrentTrack(a5)

       move.w     #SETTLEDELAY,d0
       bsr        td_delay

       movem.l    (sp)+,d2-d3
.exit:
       rts


;---------------------------------------------------------------------------
td_seekzero:
; Turn motor on. Seek track 0, reset TTETrack_CurrentTrack to 0.
; a2 = CIAAPRA
; a3 = CIABPRB
; -> d0/Z = error code (0=ok, 1=unformatted, 2=missingSectors, 3=badHeader)

       bset       #DSKSIDE,(a3)                                         ; select lower head: track 0
       bset       #DSKDIREC,(a3)                                        ; step outwards

.1:    moveq      #STEPDELAY,d0
       bsr        td_delay                                              ; wait until TRACK0 signal is valid

       btst       #DSKTRACK0,(a2)
       beq        .2

       bsr        td_step
       bra        .1

	; head is positioned over track 0 now; read it
.2:    clr.w      TTETrack_CurrentTrack(a5)
       rts


;---------------------------------------------------------------------------
td_step:
; Step a track into selected direction.
; a2 = CIAAPRA
; a3 = CIABPRB

       moveq      #STEPDELAY,d0
       bsr        td_delay
       bclr       #DSKSTEP,(a3)
       nop
       bset       #DSKSTEP,(a3)
       rts


;---------------------------------------------------------------------------
td_delay:
; Wait for ca. count * 64us.
; d0.w = count
; a0 and a1 are preserved!

.1:    move.b     VHPOSR(a6),d1
.2:    cmp.b      VHPOSR(a6),d1
       beq        .2
       subq.w     #1,d0
       bne        .1
       rts




;  unzx0_68000.s - ZX0 decompressor for 68000
;
; platon42: Modified to not preserve registers and to not use long word operations with
; unlikely and unsupported block lengths > 64 KB. get_elias inlined for speed and other
; optimizations.
;
; h0ffman: Modified to decompress an abritrary number of source bytes
;
;  in:  a0 = start of compressed data
;       a1 = start of decompression buffer
;       d6 = number of bytes in unpack buffer
;
;  out: a0 = end of compressed data
;       a1 = end of decompression buffer
;
;  trashes: d0-d2/a2
;
;  Copyright (C) 2021 Emmanuel Marty
;  Copyright (C) 2023 Emmanuel Marty, Chris Hodges
;  ZX0 compression (c) 2021 Einar Saukas, https://github.com/einar-saukas/ZX0
;
;  This software is provided 'as-is', without any express or implied
;  warranty.  In no event will the authors be held liable for any damages
;  arising from the use of this software.
;
;  Permission is granted to anyone to use this software for any purpose,
;  including commercial applications, and to alter it and redistribute it
;  freely, subject to the following restrictions:
;
;  1. The origin of this software must not be misrepresented; you must not
;     claim that you wrote the original software. If you use this software
;     in a product, an acknowledgment in the product documentation would be
;     appreciated but is not required.
;  2. Altered source versions must be plainly marked as such, and must not be
;     misrepresented as being the original software.
;  3. This notice may not be removed or altered from any source distribution.



zx0_decompress_tte:
       moveq.l    #-128,d1                                              ; initialize empty bit queue
                                ; plus bit to roll into carry
       moveq.l    #-1,d2                                                ; initialize rep-offset to 1
       addq.w     #1,d6                                                 ; add one extra read for first pass
       bra.s      zx0_literals

zx0_do_copy_offs2
       move.b     (a1,d2.l),(a1)+
       move.b     (a1,d2.l),(a1)+

       add.b      d1,d1                                                 ; read 'literal or match' bit
       bcs.s      zx0_get_offset                                        ; if 0: go copy literals

zx0_literals
        ; read number of literals to copy
       moveq.l    #1,d0                                                 ; initialize value to 1
zx0_elias_loop1
       add.b      d1,d1                                                 ; shift bit queue, high bit into carry
       bne.s      zx0_got_bit1                                          ; queue not empty, bits remain
       TTEUP
       move.b     (a0)+,d1                                              ; read 8 new bits
       addx.b     d1,d1                                                 ; shift bit queue, high bit into carry
                                ; and shift 1 from carry into bit queue

zx0_got_bit1
       bcs.s      zx0_got_elias1                                        ; done if control bit is 1
       add.b      d1,d1                                                 ; read data bit
       addx.w     d0,d0                                                 ; shift data bit into value in d0
       bra.s      zx0_elias_loop1                                       ; keep reading

zx0_got_elias1
       subq.w     #1,d0                                                 ; dbf will loop until d0 is -1, not 0
zx0_copy_lits
       TTEUP
       move.b     (a0)+,(a1)+                                           ; copy literal byte
       dbra       d0,zx0_copy_lits                                      ; loop for all literal bytes

       add.b      d1,d1                                                 ; read 'match or rep-match' bit
       bcs.s      zx0_get_offset                                        ; if 1: read offset, if 0: rep-match

zx0_rep_match
        ; read match length (starts at 1)
       moveq.l    #1,d0                                                 ; initialize value to 1
zx0_elias_loop2
       add.b      d1,d1                                                 ; shift bit queue, high bit into carry
       bne.s      zx0_got_bit2                                          ; queue not empty, bits remain
       TTEUP
       move.b     (a0)+,d1                                              ; read 8 new bits
       addx.b     d1,d1                                                 ; shift bit queue, high bit into carry
                                ; and shift 1 from carry into bit queue

zx0_got_bit2
       bcs.s      zx0_got_elias2                                        ; done if control bit is 1
       add.b      d1,d1                                                 ; read data bit
       addx.w     d0,d0                                                 ; shift data bit into value in d0
       bra.s      zx0_elias_loop2                                       ; keep reading

zx0_got_elias2
       subq.w     #1,d0                                                 ; dbra will loop until d0 is -1, not 0
zx0_do_copy_offs
       move.l     a1,a2                                                 ; calculate backreference address
       add.l      d2,a2                                                 ; (dest + negative match offset)
zx0_copy_match
       move.b     (a2)+,(a1)+                                           ; copy matched byte
       dbra       d0,zx0_copy_match                                     ; loop for all matched bytes

       add.b      d1,d1                                                 ; read 'literal or match' bit
       bcc.s      zx0_literals                                          ; if 0: go copy literals

zx0_get_offset
       moveq.l    #-2,d0                                                ; initialize value to $fe

        ; read high byte of match offset
zx0_elias_loop3 
       add.b      d1,d1                                                 ; shift bit queue, high bit into carry
       bne.s      zx0_got_bit3                                          ; queue not empty, bits remain
       TTEUP
       move.b     (a0)+,d1                                              ; read 8 new bits
       addx.b     d1,d1                                                 ; shift bit queue, high bit into carry
                                ; and shift 1 from carry into bit queue

zx0_got_bit3
       bcs.s      zx0_got_elias3                                        ; done if control bit is 1
       add.b      d1,d1                                                 ; read data bit
       addx.w     d0,d0                                                 ; shift data bit into value in d0
       bra.s      zx0_elias_loop3                                       ; keep reading

zx0_got_elias3
       addq.b     #1,d0                                                 ; obtain negative offset high byte
       beq.s      zx0_done                                              ; exit if EOD marker
       move.b     d0,-(sp)                                              ; transfer negative high byte to stack
       move.w     (sp)+,d2                                              ; shift it to make room for low byte

       TTEUP
       move.b     (a0)+,d2                                              ; read low byte of offset + 1 bit of len
       asr.l      #1,d2                                                 ; shift len bit into carry/offset in place
       bcs        zx0_do_copy_offs2                                     ; if len bit is set, no need for more
       moveq.l    #1,d0                                                 ; initialize length value to 1
        ; read rest of elias-encoded match length
       add.b      d1,d1                                                 ; read data bit
       addx.w     d0,d0                                                 ; shift data bit into value in d0

zx0_elias_loop4
       add.b      d1,d1                                                 ; shift bit queue, high bit into carry
       bne.s      zx0_got_bit4                                          ; queue not empty, bits remain
       TTEUP
       move.b     (a0)+,d1                                              ; read 8 new bits
       addx.b     d1,d1                                                 ; shift bit queue, high bit into carry
                                ; and shift 1 from carry into bit queue

zx0_got_bit4
       bcs.s      zx0_do_copy_offs                                      ; done if control bit is 1
       add.b      d1,d1                                                 ; read data bit
       addx.w     d0,d0                                                 ; shift data bit into value in d0
       bra.s      zx0_elias_loop4                                       ; keep reading
;zx0_done
       rts

zx0_done
       clr.l      TTETrack_Length(a5)                                   ; unpacking complete, ensure to stop reading any more data
       rts

TTETrackVars:
       dcb.b      TTETrack_Sizeof,-1


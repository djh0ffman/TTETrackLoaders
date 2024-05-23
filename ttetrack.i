                         RSRESET
TTETrack_CurrentTrack    rs.w       1
TTETrack_SyncMark        rs.w       1
;TTETrack_DecodeTrack     rs.w       1
TTETrack_Mode            rs.w       1
TTETrack_MFM1            rs.l       1      
TTETrack_Dest            rs.l       1
TTETrack_StartByte       rs.w       1
TTETrack_StartSector     rs.w       1
TTETrack_Length          rs.l       1
TTETrack_Sectors         rs.l       SECT_PER_TRK
TTETrack_UnpackState     rs.l       6
TTETrack_Sizeof          rs.w       0

TTEUP                    macro
                         move       sr,d5
                         subq.w     #1,d6
                         bne        .\@edge
                         lea        .\@edge(pc),a3
                         rts
.\@edge
                         move       d5,ccr
                         endm


TTEUP2                   macro
                         subq.w     #1,d6
                         bne        .\@edge
                         lea        .\@edge(pc),a3
                         rts
.\@edge
                         endm
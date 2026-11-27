; reveal-demo-recorded.g — the reveal shot captured to video.
; Same choreography as reveal-demo.g, wrapped in M3/M5 so the gimbal's
; record key is held for the duration of the moves.
;
;   cat examples/reveal-demo-recorded.g > /path/to/cranectld.fifo
;
G21
G28                    ; home (defines origin) — not recorded
M3                     ; start recording

; --- slow cinematic reveal (feedrate-driven) ---
G90                    ; absolute
G1 Y-35 F350           ; dip down to -35
G4 P500                ; hold 0.5 s
G1 X25 Y15 F190        ; crane up + pan to +25/+15 (slow)

; --- relative "wave" (G91) ---
G91                    ; relative
G1 X-6 F300
G1 X6  F300
G1 X-6 F300
G90                    ; back to absolute

; --- rapid (G0) then a controlled sweep (F) ---
G0 X-30 Y-20           ; fast jump
G4 P300
G1 X30 Y-20 F450       ; smooth sweep across

; --- machine vs user frames ---
G53 X0 Y0              ; raw firmware datum (ignores origin)
G4 P400
G1 X0 Y0               ; back to home-relative origin
G92 X0 Y0              ; declare current pose user-zero
G0 X0 Y0

M5                     ; stop recording
G28                    ; re-home (not recorded)
M2

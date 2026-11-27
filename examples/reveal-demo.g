; reveal-demo.g — a short cinematic tour of cranectld's G-code set.
; All moves stay within ±35 tilt / ±30 pan. Feed it to a running daemon:
;
;   cat examples/reveal-demo.g > /path/to/cranectld.fifo
;
G21                    ; (ignored) units parse cleanly
G28                    ; gimbal home -> defines user origin 0,0

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

G28                    ; return to gimbal home
M2

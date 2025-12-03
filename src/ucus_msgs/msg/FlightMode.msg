uint8 mode
uint8 MC=0                    # Multicopter hover mode
uint8 VTOL_TAKEOFF=1          # Vertical takeoff mode
uint8 HOVER=2                 # Hover/position hold mode
uint8 TRANSITION_MC_TO_FW=3   # Transition (Multicopter → Fixed-Wing)
uint8 FW=4                    # Fixed-wing forward flight
uint8 TRANSITION_FW_TO_MC=5   # Transition (Fixed-Wing → Multicopter)
uint8 LAND=6                  # Vertical or gliding landing
uint8 EMERGENCY=7             # Failsafe / emergency mode

uint8 reason
uint8 REASON_MANUAL=0         # Operator requested mode change
uint8 REASON_MISSION=1        # Automatic change from mission logic
uint8 REASON_FAILSAFE=2       # Sensor, link, or safety-triggered change




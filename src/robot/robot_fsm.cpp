/*

Pseudo code for the FSM and other functions:

'AT' = 'April Tag' 

1. GET TO RAMP
    Drive toward "start_area_tag" until (delta_z to AT6) <= __________
    Turn right 90 deg
    [opt: check if centered on ramp and realign() if necessary]
    Drive toward "ramp_A_base_marker" until (delta_z to AT4) = __________ +- tolerable error
        (needs to work with Romane's code)

2. RAMP (unloaded) --> use Romane's hardcoded sequence
    Drive up and rev down (x2)
    Drive up, turn left 90 deg, rev down other side


3. GET TO TRAY (unsure about implementation)
    either: 
    - Turn 180 deg and drive forward until (delta_z to AT1 or AT2) <= __________
                or until (yaw to AT1 or AT2) ~ ________
    or
    -  Reverse until AT2 visible and (delta_z to AT2) <= __________
                or until (yaw to AT2) ~ ____90ish deg____
        * prob not enough room

    and then turn 90 deg (right if fwd, left if rev)

4. PICK UP TRAY --> use Romane's hardcoded sequence
    (ex. gripOpen(), reach up and drive, touch table and push fwd, gripClose(), get to rest pos)
    
5. RAMP (loaded) --> use hardcoded sequence, but maybe do just once
    Config arms: Theta_1 = PI/2 ; Theta_2 = -ypr.roll
    Turn around and drive toward "right_of_ramp)_tag" until (delta_z to AT3) = __________ +- tolerable error
    
    
6. GET TO TABLE
    Drive toward "side_area_tag" until (delta_z to AT9) <= __________
    Turn 90 deg
    [opt: check if centered on AT5 and realign() if necessary]
    Drive toward "ramp_A_base_marker" until (delta_z to AT4) = __________ +- tolerable error
        (needs to work with Romane's code)

    

7. PLACE TRAY --> Romane's pickup code but in reverse (kinda)
    (ex. lift, drive, stop and lower to proper height, openGrip(), return to rest pos)

8. DISHWASHER TRAY (repeat steps 3-6 for Dishwasher tray)
    *** note: dishwasher height != table height ; 
            Nav to AT6 (turn R90, drive, turnL90)
            Pick up and turn 180 deg
            Use AT6 instead of AT5 ; 
            Use AT7 for drive forward, turn 90, realign and/or drive fwd






function realign (Delta_X, Delta_Zf, Delta_Zb) 
** takes in a goal horizontal shift, the traversable space in front and behind the robot
    and tries to align with that target Delta_X. 

    // if Delta_X small, x = 1, otherwise x = smaller chunks (div into Delta_X/x_max chunks)
    // Maybe change delta_t's based on target Delta_X
    loop x times:

    if (Delta_Zf > threshold) { => there is enough room in front
        *** S shape ***
        Drive (mod w/ turn) for delta_t
        Drive (mod w/ -turn) briefly to staighten out 
        
        } else if (Delta_Zb > threshold) { => there is enough room in back
        
        *** backward first, then reassess ***
        
        
        *** OR backward first, then S shape ***
        Reverse (turn=0) for delta_t
        Drive (mod w/ turn) for delta_t
        Drive (mod w/ -turn) briefly to staighten out 

        *** OR backwards S shape ***
        Reverse (same a) for delta_t
        Drive (mod w/ -turn) briefly to staighten out 

    } else { => no room in front or back ()
        
        *** turn 90 deg, move, turn -90 deg, try again ***

        *** OR sit and wait for joystick ctrl & cmd to reeval ***
            -- maybe timeout and reeval
    }
}

*/
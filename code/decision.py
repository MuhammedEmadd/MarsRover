import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # if the rover is spinning with moving forward  
    if Rover.timer==1 and time.time() - Rover.spinning_time > (Rover.max_spinning_time + 1): 
        Rover.throttle = 0.2
        Rover.brake = 0
        Rover.steer = 0
        Rover.timer=0
        Rover.spinning_time=0
        #if the requirements are met 
        #initiate state returning home
    if Rover.samples_collected >= 5 and Rover.mapped_percentage >= 95.0:
        print('RETURNING HOME')
        if abs(Rover.pos[0] - Rover.start_pos[0]) < 10 and abs(Rover.pos[1] - Rover.start_pos[1]) < 10:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            print('RETURNED HOME!!!! BEAM ME UP!!!')
            return Rover

    # Rover is stuck so try to get unstuck
    if Rover.mode == 'stuck':
        print('STUCK!! Lets get out !!!!!!!!')
        if time.time() - Rover.stuck_time > (Rover.max_stuck + 1):
            Rover.mode = 'forward'
            Rover.stuck_time = time.time()
        else:
            # Perform evasion to get unstuck
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15
        return Rover



    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if abs(Rover.steer)==15 and Rover.vel>0 and Rover.timer==0:
                # If the velocity is still 0 after throttle, it's stuck
                Rover.spinning_time = time.time()
                Rover.timer=1
                return Rover
            # we are about to get stuck
            if Rover.vel < 0.2 and Rover.throttle != 0:
                # If the velocity is still 0 after throttle, it's stuck
                if time.time() - Rover.stuck_time > Rover.max_stuck:
                    # Initiate stuck mode after 5 seconds of not moving
                    Rover.mode = 'stuck'
                    return Rover
            else:
                # Reset stuck time
                Rover.stuck_time = time.time()
            if Rover.sample_seen:
                if Rover.picking_up != 0:
                    print('SUCCESSFULLY PICKED UP SAMPLE')
                    Rover.samples_collected += 1
                    # Reset sample_seen flag
                    Rover.sample_seen = False
                    Rover.sample_timer = time.time()
                    return Rover
                # Timeout finding a sample
                if time.time() - Rover.sample_timer > Rover.sample_max_search:
                    print('UNABLE TO FIND SAMPLE IN TIME LIMIT')
                    Rover.sample_seen = False
                    Rover.sample_timer = time.time()
                    return Rover
                avg_rock_angle = np.mean(Rover.rock_angle * 180/np.pi)
                if -15 < avg_rock_angle < 15:
                    # Only drive straight forward for sample if it's within 15 deg
                    print('APPROACHING ROCK HEAD ON')
                    if max(Rover.rock_dist) < 15: 
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = avg_rock_angle
                    else:
                        # Set throttle at half normal speed during approach
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = avg_rock_angle
                elif -50 < avg_rock_angle < 50:
                    print('ROTATING TO SAMPLE: ', avg_rock_angle)
                    if Rover.vel > 0 and max(Rover.rock_dist) < 40: #50
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    
                    #elif time.time() - Rover.spinning_time > Rover.max_spinning_time:
                        #Rover.steer = 0
                        #Rover.brake = 0
                        #Rover.throttle = Rover.throttle_set
                    
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = avg_rock_angle/4 #avg_rock_angle/6
                else:
                    # Keep the logic simple and ignore samples +/-13 degrees
                    print('LOST SIGHT OF THE SAMPLE')
                    Rover.sample_seen = False
             
            #elif time.time() - Rover.spinning_time > Rover.max_spinning_time:
                #Rover.steer = 0
                #Rover.brake = 0
                #Rover.throttle = Rover.throttle_set
                
                
            #elif max(Rover.nav_dists) >= Rover.stop_forward and len(Rover.nav_angles) > 50:  
            #if we found a terrain ahead
            elif len(Rover.nav_angles) > 50:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),
                                              -15, 15)
                Rover.spinning_time = 0
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Rover is stopped with vision data; see if there's a path forward
                #if max(Rover.nav_dists) < Rover.go_forward and len(Rover.nav_angles) < 100:
                if len(Rover.nav_angles) < 100:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line
                    #   will induce 4-wheel turning
                    Rover.steer = -15
                # Stopped; see if sufficient navigable terrain in front then go
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),
                                          -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something even if no modifications have been
    #    made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.sample_seen = False

    return Rover
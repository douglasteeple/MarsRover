import numpy as np
import time
from perception import to_polar_coords, distance, radians_to_degrees, timestampstring

#
# im stuck if i can't move but there are plenty of navigable pixels in sight
#
def im_stuck(Rover):
    return (Rover.stuck_count>40)

#
# direction to the nearest rock, a good choice if we can't decide which way to go
#
def nearest_rock_angle(Rover):  # in radians
    min_dist = 1000.
    min_dist_idx = -1
    for idx in range(len(Rover.samples_pos[0]) - 1):
        test_rock_x = Rover.samples_pos[0][idx]
        test_rock_y = Rover.samples_pos[1][idx]
        rock_sample_dists = distance([test_rock_x,test_rock_y],Rover.pos)
        if rock_sample_dists < min_dist:
            min_dist = rock_sample_dists
            min_dist_idx = idx
    dists, angles = to_polar_coords(Rover.samples_pos[0][min_dist_idx], Rover.samples_pos[1][min_dist_idx])
    if min_dist_idx >= 0:
        return angles
    else:
        return 0

def balance_to_right(Rover):
    return np.mean(Rover.nav_angles) > 0.

def closeto(this, that, howmuch=0.1):
    return (np.abs(np.abs(this)-np.abs(that)) < howmuch)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # exit if we are home
    if Rover.back_home == True:
        return Rover
    
    # stop if we are near a sample
    if ((Rover.near_sample == 1) & (Rover.pickup_started == 0)):
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.mode = 'stop'
        #print(timestampstring(),'stopping to pickup')
    
    # restart if a pickup finished
    if (Rover.pickup_started == 1) & (Rover.near_sample != 1):
        Rover.mode = 'forward'
        Rover.brake = 0
        Rover.throttle = Rover.throttle_set
        Rover.pickup_started = 0
        Rover.rocks_picked_up += 1
        Rover.pickup_location = Rover.pos
        #print(timestampstring(),'restarting')

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # At the goal?
                if (len(Rover.goal) > 0) & (Rover.success == True):
                    dists, angles = to_polar_coords(Rover.goal[0], Rover.goal[1])
                    np.append(Rover.nav_angles,[angles])
                    #print('steering perferentially towards goal dist=', dist)
                    if (distance(Rover.pos, Rover.goal) < 3.0):
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
                        Rover.back_home = True
                        print(timestampstring()," Goal reached.... stopping.")

                # stuck?
                if im_stuck(Rover) | (Rover.near_sample == 1) | (Rover.success == True):  # don't add the steering bias when stuck or near a rock or coming home
                    totalmean = np.mean(radians_to_degrees(Rover.nav_angles))
                else:
                    totalmean = np.median(radians_to_degrees(Rover.nav_angles))
                    if totalmean < 0.5:
                        totalmean = np.mean(radians_to_degrees(Rover.nav_angles)+Rover.steering_bias)
                    nearest_rock = radians_to_degrees(nearest_rock_angle(Rover))
                    #totalmean = np.mean([totalmean,nearest_rock/10])
                    #print('nearest rock angle in degrees',nearest_rock)
                
                # steer preferentially towards a rock
                if Rover.rock_angles is not None:
                    if len(Rover.rock_angles) > 0:
                        totalmean = np.mean([totalmean/20, radians_to_degrees(Rover.rock_angles[0])])
                        #print(timestampstring(),'steering towards rock 1', Rover.rock_dists[0])

                Rover.steer = np.clip(totalmean, -15, 15)
                #Rover.pid.setPoint(np.clip(totalmean, -Rover.max_steer, Rover.max_steer))
                #Rover.steer = Rover.pid.update(Rover.steer)
                
                # check for stuck
                if Rover.pickup_started == 0:
                    if Rover.near_sample == 0:
                        if (Rover.vel < 0.2) & (Rover.vel >= 0.0):
                            if distance(Rover.pickup_location, Rover.pos) > 1.0:
                                Rover.stuck_count += 1
                                if Rover.stuck_count == 1:
                                    Rover.stuck_time = time.time()
                                if im_stuck(Rover):
                                    Rover.mode = 'backward'
                                    print(timestampstring(),"Stuck",Rover.stuck_count,Rover.steer,Rover.brake,Rover.throttle,Rover.vel)
            
                # are we in stuck mode but moving? then we got unstuck
                if im_stuck(Rover):
                    if Rover.vel >= 0.2:
                        Rover.stuck_count = 0
                        Rover.mode = 'forward'
                        print(timestampstring(),"UnStuck",Rover.stuck_count,Rover.steer,Rover.brake,Rover.vel)
        
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    #Rover.steer = 0
                    Rover.mode = 'stop'

        # we're stuck, back out...
        if Rover.mode == 'backward':
            if (Rover.backup_count > Rover.backup_threshold) | ((Rover.backup_count>(Rover.backup_threshold/2)) & closeto(Rover.vel, 0.0, 0.2)):
                Rover.backup_count = 0
                Rover.stuck_count = 0
                Rover.steer = -Rover.max_steer
                if balance_to_right(Rover):
                    Rover.steer = -Rover.max_steer   # turn right
                Rover.mode = 'stop'
                Rover.throttle = 0
                print(timestampstring(),"Stopping",Rover.backup_count,Rover.steer,Rover.brake,Rover.throttle,Rover.vel)
            else:
                Rover.steer = Rover.max_steer
                if balance_to_right(Rover):
                    Rover.steer = Rover.max_steer  # turn left
                Rover.brake = 0
                Rover.throttle = -Rover.throttle_set
                Rover.backup_count += 1
                if Rover.backup_count == Rover.backup_threshold:
                    print(timestampstring(),"Backing up",Rover.backup_count,Rover.steer,Rover.brake,Rover.throttle,Rover.vel)

        # If we're already in "stop" mode then make different decisions
        if Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                #???Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if (Rover.steer != -Rover.max_steer) & (Rover.steer < 0.):
                        Rover.steer = Rover.max_steer
                    elif (Rover.steer != Rover.max_steer) & (Rover.steer > 0.):
                        Rover.steer = -Rover.max_steer
                    else:
                        Rover.steer = -Rover.max_steer
                
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        print('just do something')

    # pickup sample if nearby
    if ((Rover.near_sample == 1) & (Rover.pickup_started == 0)):
        Rover.pick_up = 1
        Rover.pickup_started = 1

    return Rover


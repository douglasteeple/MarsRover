# audacity-robotics
## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

First I took a series of images from the simulator and stored them in dataset/IMG/. I then stepped through the notebook
process on the test data given and then on the images I took.

The first modifications were in perspect_transform, simply using the process learned in class.

Next, I added two functions `color_match` and `color_inverse_thresh`. Color match took the input color and compared the difference
against a threshold. Simple but it worked for this example. One problem with it is that the color should be recognized whether
it is bright or dark, so it should have some kind of normalization. I added `color_inverse_threshold` to check that a color is
less than the given color. It is used to detect walls. I could not use the given `color_thresh` because it does not distinguish 
between black and white, both having similar rgb profiles.


#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

I modified `process_image` as per the steps requested:

* I added the source and destination points as per the class material.
* I then did the perspective transform using `perspect_transform` and the source and destination to create the warped image.
* I then did three color selections.
  - color selection for navigable areas used color_thresh using the values rgb=(170, 170, 170) to pick the navigable path.
  - inverse color selection using rgb=(100,100,100) to select the walls.
  - color match using rgb=(150,150,0) (yellow) to select the rocks. Note: This was later changed to (120,120,0) to account for rocks that were a little darker than most, though still yellow.
* I converted to rover coordinates using `rover_coords` on the color selected images.
* And finally mapped to the world coordinates using `pix_to_world` given the rover coordinates, the Rover position and yaw
and the worldmap size and scale.

I ran it twice, once on the given test data (`test_mapping.mp4`) and on my sample data which included a rock (`doug_mapping.mp4`).
The movie files can be found in the `output` directory.

[image3]: ./output/test_mapping.mp4
[image4]: ./output/doug_mapping.mp4

<center>
### Mapping Movie Output
<table>
    <tr>
        <th>Test Output</th><th>Doug's Output</th>
    </tr>
    <tr>
        <td>
            <iframe height="240" src="./output/test_mapping.mp4" allowfullscreen></iframe>
        </td>
        <td>
            <iframe height="240" src="./output/doug_mapping.mp4" allowfullscreen></iframe>
        </td>
    </tr>
</table>
</center>

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

## 1. The Perception Step

 * I took the basic code from `process_image`, but added the `Rover.Vision_image` steps to record the inset images.
    The basic steps for the three color selections are as per `process_image`.
 * Additionally the xpix, and ypix coordinates from the `Rover.Vision_image` were used as argument to the rover_coords
    function to get rover-centric coordinates, which were then passed to `pix_to_world` to add planes to the worldmap.
    Navigable coordinates were added to the blue plane, obstacle coordinates to the red plane and rock coordinates to
    the green plane.
 * The xpix and ypix Rover-centric coordinates were then passed to the `to_polar_coords` fundtion to get the distances
    and angles.
 * I added and additional field to the Rover data to record the rock distances and angles and calculated these as per
    the previous step, but using the Rover-centric xpix and ypix of the **rocks**. The rock angles are used in the `decision_step`
    to move towards a rock that is seen in order to pick it up.

All of the result images can be viewed in the notebook.


## 2. The Decision Step

    Most of the changes were in the decision step. I added the following functionality:

  a) Turning towards rocks.

  b) Picking up rocks.

  c) Checking for getting stuck situations - where plenty of pixels are seen to move ahead but a wheel was stuck.

  d) A little smarter turning decision process, where the direction to turn (after stopping to pick up a rock) is
           opposite to the prior turning direction.

  e) Adding a steering bias to try to hug one side of a wall.

  f) Adding a steering weight function (equivalent to the h heuristic in the A* algorithm) to direct the robot
           towards home (the starting point) if the goal has been achieved.

  g) Adding a success check, indicating that the goal has been achieved and the robot has returned home. The
            new field back_home is set and the Rover stopped.
  h) Added `backup` Rover mode for the case that the Rover gets stuck (i.e. velocity is zero) but there are plenty of navigable pixels. This happens when the Rover encounters small rocks in the path.

Implementation:

        a) Moving towards rock was accomplished by adding the rock angle as calculated in the perception step to the mean
        angle calculation. The mean of the steering angle was reduced by a factor of 10 to have steering towards a nearby
        rock take precedent over the normal mean angle.

        b) Picking up rocks was accomplished by testing the `near_sample` flag and adding two new flags to the Rover data structure.
        The flags are `pick_up`, indicating that a pickup was signalled and `pickup_started` indicating that the pickup process
        had been initited. If the robot is near a sample but the `pickup_started` is false, then a stop process is started.
        If `pickup_started` is true but `pickup_started` is false then a restart process is initiated, `pickup_started` is set to
        false and the count of rocks picked up incremented. The new field `rocks_picked_up` indicating the number of rocks
        successfully picked up is used in part to determine the goal had been acheived (see Other Changes below).

        c) Getting Unstuck
        Sometimes a path was visible to the Rover but a wheel was stuck on a wall. I added an unstuck step that if the
        extent of navigable terrain was greater than `stop_forward` but the velocity was near zero for more than 25 data
        cycles, the Rover would initiate a turn similar to the turn when there is lack of navigable terrain pixels.

        d) A little smarter turning involved changing the turn angle to the opposite sign of the prior angle. This enabled the Rover
        to restart successfully after picking up rocks that are usually at the edge of a wall.
        
        e) The steering bias was added as a field steering_bias to the Rover data structure which is added to the total mean
        of the calculated angles. This bias caused the Rover to hug a wall, and had the interesting side effect of causing
        steering oscillation. At first I thought that was bad but as I watched the Rover in action, it created a kind of
        side to side scanning effect which made the Rover better at detecting rocks on either side of the wall as it traveled.

        f) The steering weight is a kind of goal heuristic. If the goal (as per Other changes below) is met, then the heuristic
        is appended to the nav_angles. The heuristic is calculated as the angle to the goal. The goal position is recored in 
        a new Rover field at startup. The effect of adding gloa angle to the set of angles is that when a decision is made
        to go right or left then the decision is biased towards heading to the goal.

        g) The success check uses the check for being near the goal point and having acheived the success criteria as in  
        (Other Changes) below. The Rover stops, and flashes a message on the screen. Yahoo!

        h) The backup mode is triggered when navigable rocks are visible but there is no forward velocity for
        40 data samples. When this condition is met (and the Rover is not stopped picking up a rock) the mode
        is changed to 'backward' and the throttle set to a negative value. The Rover steers counter to the prior
        angle and drives backwards for `Rover.backup_threshold` data cycles. It then stops, steers in the 
        opposite direction and goes forward again.

## 3. Other Changes
 - `supporting_functions`: A step was added to check that the goal has been achieved, which sets the new Rover field `success`
    to true `if (perc_mapped>40) & (fidelity>60) & (Rover.rocks_picked_up>=6)`


## 4. Tips Implemented and Further Work Needed

    `Optimizing Map Fidelity`: I had no trouble with fidelity. Tweaking the `brake_set`, and the `throttle_set` values gave adequate fidelity numbers.

    `Optimizing Time`: I adjusted the maximum velocity and throttle to get adequate results while not compromising bypassing rocks or lowering fidelity. Returning all 6 rocks took quite a while as paths were retraced. However
        time was not a project submission requirement.
    
    `Optimizing % Mapped`: I did not implement a `"do not retrace paths already taken"` algorithm. I kind of got stuck on this one for two reasons. One, if the Rover goes down a blind alley, how would it get back? Two, if a rock ws missed for some reason, and the path was not retraceable then the rock would never be found.

    `Optimizing for Finding All Rocks`: I implemented a `stay close to the wall` algorithm by adding in a steering bias
    to the mean angle calcuation. In general I found that `hug left` worked better than `hug right`.

    The color matching algorithm is weak, it has trouble with saturation. As you get further along in picking rocks you
    see that some are darker yellow. The existing color has been tuned to work, but really a better `distance` algorithm 
    should be implemented.

## 5. Results

Here are two results shown as videos where the robot picks up 3 rocks, and brings them back home, and stops and declares that the goal has been
reached. The given goal was >40% mapped, >60% fidelity, at least one rock picked up, and returned within 3 meters of the
original start position. 

There are also two static images showing autonomous pickup of 5 rocks and pickup and return to home of all 6 rocks. I should note that
the results are not 100% reproducible depending on the starting position and where the rocks are in a particular run.

I took the videos as screen recording rather than as a recording saved from the command line because the screen
recording shows more information such as the info panel insets. The command line record only shows
the Rover view.

I experimented with using the median nav angle rather than the mean in decision.py and found I got better fidelity and the Rover got stuck less.

I also experimented with a search optimization that used an A* type weighting that added the angle to the nearest rock to the nav angles. This however did not work as it caused too much instability in steering. One thought was to smooth the steering angle with PID smoothing but I did not get a chance to debug it.


[image5]: ./output/NDRobot2.m4v
[image6]: ./output/6RocksNoGoal
[image7]: ./output/5Rocks.png

<hr>
<center>
### Example Movie Output from Rover Simulator
<table border=1>
    <tr>
        <th>3 rocks picked up and returned to goal</th><th>6 rocks, but no goal</th>
    </tr>
    <tr>
        <td>
            <p align=center><iframe height="240" src="./output/NDRobot2.m4v" allowfullscreen></iframe></p>
        </td>
        <td>
            <p align=center><iframe height="240" src="./output/6RocksNoGoal.m4v" allowfullscreen></iframe></p>
        </td>
    </tr>
    <tr>
        <td>
            <p align=center>A very early run from May 28. In this run the return to goal worked well but wall hugging had not yet been implemented. The goal of returning 6 rocks was relaxed to 3.</p>
        </td>
        <td>
            <p align=center>In this run from May 30 wall hugging was working, and the goal was picking up 6 rocks, but return to goal failed.</p>
        </td>
    </tr>
    <tr>
        <th>5 Rocks Picked up</th><th>6 Rocks Picked up and At Home!</th>
    </tr>
    <tr>
        <td>
            <p align=center><img height="311" width="400" src="./output/5Rocks.png"/></p>
        </td>
        <td>
            <p align=center><iframe height="240" src="./output/6RocksGoal2.m4v" allowfullscreen></iframe></p>
        </td>
    </tr>
    <tr>
        <td>
            <p align=center>May 29. The first run getting 5 rocks, with wall hugging working for the first time.</p>
        </td>
        <td>
            <p align=center>May 30. All 6 rocks! And a return to goal! The entire run is autonomous.</p>
        </td>
    </tr>
</table>
</center>
<hr>
#### Note that the Rover stops and declares victory (`Yahoo! in yellow`) after achieving the task requirements and returning to the home position.





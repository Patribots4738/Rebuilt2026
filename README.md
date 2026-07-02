
[![Game Manual](https://i.ytimg.com/vi/0JaguGctF1A/maxresdefault.jpg)](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

<br />The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 


This repository is entirely student-created and maintained.

We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, & be sure to star this repository if you find anything helpful or interesting!

### [See how we did!](https://www.statbotics.io/team/4738)

## ✨ Highlights ✨ 
- Automatic shooter tracking to three differnt targets based on robot location
- Turret tracking using trigonometry
- Shooting while driving using projected position and _time of flight_
- Heavy _April Tag_ filtering for precise _Hub_ tracking
- Automatic stow hood when under the _Trench_
- Log replay using _Advantage Scope_ and _Advantage Kit_
- Field centric inverted gear swerve drive

## Simulation and Testing

### PID Tuning with Elastic

### Log Replay Using AdvantageKit

## Swerve Drive
We use `MK5N` swerve modules from `Swerve Drive Specialties` with a `Pigeon 2.0` as our IMU.

### Field Centric
> To make the robot more user-friendly for the driver, our swerve drive is field-centric, but can be operated robot relative. When disabled, we use `MegaTag 1` on our `Limelights` to obtain our starting orientation on the field. We then switch to `MegaTag 2` when enabled to more stable pose estimation and use `April Tags` that are farther away, relying on our `Pigeon 2.0` gyroscope to monitor changes in our orientation.

### Defensive X Pose
> When our robot is stationary in teleop, we set the position of our wheels to an X. In this X-pose, our robot will stay stationary, even if another robot drives into us at full speed. This idea was first mentioned to us by [3255 - SuperNURDS](<https://github.com/FRCTeam3255>) and has helped us tremendously!

### Discretize
> When turning while driving, a dt of 0.02s isn't small enough to get precise movement that doesn't drift. To remedy this, we discretize our speeds over 0.02s to get a new trajectory that results in the robot being able to spin while driving without drifting left or right. This is critical to our autonomous, since we need our position to be as accurate as possible. You can learn more about the evolution of it here: [Swerve Drive Skew and Second Order Kinematics - Chief Delphi](<https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/33>). This code is now implemented into WPILib's `ChassisSpeeds` library with `ChassisSpeeds.discretize(ChassisSpeeds currentSpeed, double dtSeconds)`!

### Wheel Characterization
> Calculating our odometry is based on the radius of our wheels, which changes throughout the season as the tread wears out. With the `Pigeon 2.0`, and inspiration from [6328 - Mechanical Advantage](<https://github.com/Mechanical-Advantage>), we can measure when one full turn of the robot has been completed, then use the distance traveled by each wheel to find the wheels' radius, helping us maintain the accuracy of our odometry.

## Autonomous
### Autonomous Routines
> We use `PathPlanner` to construct a modular autonomous. In `PathPlanner`, we use named waypoints, scheduled commands, & bezier curves to generate a singular auto path between a starting position, preferable shooting position, or note location. We then link multiple auto paths together to make one predetermined autonomous. Using note detection & logic, we are able to make quick decisions on whether or not we got the note, or if we should skip trying to get a note that isn't there.

### Multitarget Shooter Tracking & Automated Switching
### Automatic Hood Stow

## Teleoperated 
### Joystick Axis Remapping
> We use Xbox controllers to control the robot. However, the range of the joystick axis from the Xbox controller is the shape of a skewed square with chamfered edges which is preferable for usage. In PatriBoxController, we remapped the range of input to a circle that is easier to use. Here's our [desmos](https://www.desmos.com/calculator/e07raajzh5) if you want to check out the math!

### Shooting While Driving
### Heavy April Tag Filtering
### Adjusting Shot Offset During a Match

## Major Class Functions 

## Controls


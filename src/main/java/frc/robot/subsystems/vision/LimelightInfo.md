# Limelight Info

hihi twins

## General Info

All Limelights should be configured to use a few common settings.
- **Netmask:** should be `255.255.255.0`
- **Team Number:** should be `4738` (obviously)
- **Gateway:** should be `10.47.38.1`
- **Static IP:** should be 10.47.38.XX, where XX should be one more than the last camera in this list and should start at 11.
- **Exposure:** 65
- **Black Level Offset:** 5
- **Sensor Gain:** 4.8
- **Full 3D Targeting:** Must be **enabled** for pose estimation.

> For AdvantageScope logging, the cameras will appear as CameraX.
> X will be the order of initialization of Vision in RobotContainer.java.
> The order should be from top to bottom, and skip any Limelights not on the robot.
> Camera numbers are dynamic, not static. They will change based on order entered when creating a new Vision object.


### Temperatures

On both Limelight 4 and 3g, the absolute limits of temperatures are -20°C to 85°C.
However, for best performance, keep the temperature in the range of 0°C to 75°C.
Upon reaching above 75°C, the Limelight will automatically reduce its performance to prevent damage.
After reaching 85°C, the Limelight will force shut down due to overheating, and may cause permanent damage.
Start attempting to cool down the Limelight if it reaches 60°C, be it powering it off or using a fan.
The Limelight 4s on our bot have cooling pads, so they should be ok, however, the 3g is most likely to overheat due to not having a cooling fan.

Max temperature before shutdown: 85°C<br>
Recommended max temperature: 75°C<br>
Warning temperature: 60°C<br>
Min temperature: -20°C<br>
Recommended min temperature: 50°C<br>

## Cameras

### limelight-threeg
> Mounted on the right side of the robot, closest to the hopper.

Code name: limelight-threeg<br>
Informal name: Linda<br>
Static IP: 10.47.38.11<br>
Model: Limelight 3G<br>
Software Version: 2026.0<br>
Notes: Hadizah's favourite

### limelight-right
> Mounted on the right side of the robot, on top of the rear right swerve module.

Code name: limelight-right<br>
Informal name: Xiao'Zhao (Zhao for short)<br>
Static IP: 10.47.38.12<br>
Model: Limelight 4<br>
Software Version: 2026.0<br>
Notes: My 2nd favourite <3

### limelight-hailo
> Mounted on the left side of the robot, on top of the rear left swerve

**--Currently off-robot--**

Code name: limelight-hailo<br>
Informal name: Dialyn<br>
Static IP: 10.47.38.13<br>
Model: Limelight 4<br>
Software Version: 2026.0<br>
Notes: Has Hailo-8 accelerator. Also my favourite <3

### limelight-left
> Mounted on the left side of the robot, closest to the hopper.

Code name: limelight-left<br>
Informal name: Megatron<br>
Static IP: 10.47.38.14<br>
Model: Limelight 4<br>
Software Version: 2026.0<br>
Notes: N/A
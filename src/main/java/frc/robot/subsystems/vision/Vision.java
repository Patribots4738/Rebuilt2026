// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.OIConstants.DriverMode;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.PoseCalculations.ZONE;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {

    private final VisionIOInputsAutoLogged[] inputs;
    private final VisionIO[] cameras;
    @AutoLogOutput (key = "Subsystems/Vision/Calibration/MT1Override")
    private boolean useMT1Override = false;
    @AutoLogOutput (key = "Subsystems/Vision/Calibration/EnabledThrottle")
    private boolean enabledThrottleOverride = false;
    @AutoLogOutput (key = "Subsystems/Vision/Calibration/DisableFour")
    private boolean disableFour = false;

    // private final LoggedTunableNumber xyStdsDisabled = new LoggedTunableNumber("Vision/xyStdsDisabled", 0.001);
    // private final LoggedTunableNumber radStdsDisabled = new LoggedTunableNumber("Vision/RadStdsDisabled", 0.002);
    // private final LoggedTunableNumber xyStdsMultiTagTelopX = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopX", 0.002);
    // private final LoggedTunableNumber xyStdsMultiTagTelopY = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopY", 0.003);
    // private final LoggedTunableNumber xyStds2TagTelopX = new LoggedTunableNumber("Vision/xyStds2TagTelopX", 0.005);
    // private final LoggedTunableNumber xyStds2TagTelopY = new LoggedTunableNumber("Vision/xyStds2TagTelopY", 0.008);
    // private final LoggedTunableNumber xyStds2TagAutoX = new LoggedTunableNumber("Vision/xyStds2TagAutoX", 0.014);
    // private final LoggedTunableNumber xyStds2TagAutoY = new LoggedTunableNumber("Vision/xyStds2TagAutoY", 0.016);
    // private final LoggedTunableNumber radStdsMultiTag = new LoggedTunableNumber("Vision/RadStdsMultiTag", Units.degreesToRadians(2));
    // private final LoggedTunableNumber xyStds1TagLargeX = new LoggedTunableNumber("Vision/xyStds1TagLargeX", 0.015);
    // private final LoggedTunableNumber xyStds1TagLargeY = new LoggedTunableNumber("Vision/xyStds1TagLargeY", 0.033);
    // private final LoggedTunableNumber radStds1TagLarge = new LoggedTunableNumber("Vision/RaStds1TagLarge", Units.degreesToRadians(7));
    // private final LoggedTunableNumber minSingleTagArea = new LoggedTunableNumber("Vision/minSingleTagArea", 0.14);

    private final SwerveDrivePoseEstimator poseEstimator;

    @AutoLogOutput (key="Subsystems/Vision/RotationUpdated")
    private boolean rotationUpdated = false;

    public Vision(SwerveDrivePoseEstimator poseEstimator, VisionIO... io) {
        int cameraCount = io.length;
        cameras = new VisionIO[cameraCount];
        inputs = new VisionIOInputsAutoLogged[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            cameras[i] = io[i];
            inputs[i] = new VisionIOInputsAutoLogged();
        }
        this.poseEstimator = poseEstimator;
        
    }

    @Override
    public void periodic() {
        boolean shouldUseMT2 = false;
        double currentYawDegrees = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        for (int i = 0; i < cameras.length; i++) {
            VisionIO camera = cameras[i];

            if ((Robot.gameMode == GameMode.DISABLED && !DriverStation.isFMSAttached() && OIConstants.DRIVER_MODE != DriverMode.CALIBRATION) || (OIConstants.DRIVER_MODE == DriverMode.CALIBRATION && (!enabledThrottleOverride || disableFour && i == 0))) {
                // If robot is disabled, only process some frames
                // This should minimize overheating issues with the LL4
                camera.setThrottle(CameraConstants.DISABLED_THROTTLE);
            } else {
                // If robot is enabled, process every frame
                camera.setThrottle(CameraConstants.ENABLED_THROTTLE);
            }


            camera.setUseMegaTag2(shouldUseMT2);
            camera.setRobotOrientation(currentYawDegrees);
            camera.updateInputs(inputs[i]);
            Logger.processInputs("SubsystemInputs/Vision/Camera" + i, inputs[i]);

            if (Robot.gameMode == GameMode.DISABLED) {
                camera.setUsedTags(CameraConstants.ALL_TAGS);
            }
            else {
                camera.setUsedTags(CameraConstants.OUTPOST_TAGS);
            }


            // if (Robot.gameMode == GameMode.AUTONOMOUS) {
            //     camera.setUsedTags(CameraConstants.ALL_TAGS);
            // }
            // else {
            //     camera.setUsedTags(CameraConstants.HUB_TAGS);
            // }

            // Logger.recordOutput("Subsystems/Vision/Camera" + i + "MT1Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four").pose);
            // Logger.recordOutput("Subsystems/Vision/Camera" + i + "MT2Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four").pose);


            //when pose calcs are done fix this to align to starting line and ramp
          //  if (alignmentSupplier.get() == AlignmentMode.RAMP) {
           //     int reefTag = closestReefTag();
                // Logger.recordOutput("Subsystems/Vision/ClosestReefTag", reefTag);
            ///    camera.setUsedTags(new int[] { reefTag });
           // } else {
             //   camera.setUsedTags(FieldConstants.REEF_TAGS);
           // }
        }

        updatePoseEstimator();
    }
    
    /**
     * Finds the most reliable camera depending on how far the tags are
     * from the center of the FOV
     * 
     * @return index of the camera we want to use
     */
    private int chooseCamera() {
        double minTagDistance = Integer.MAX_VALUE;
        int camIndex = 0;

        for (int i = 0; i < cameras.length; i++) {
            if (cameras[i].getAverageTagDistance() < minTagDistance) {
                minTagDistance = cameras[i].getAverageTagDistance();
                camIndex += i;
            }
        }

        return camIndex;
    }

    private void updatePoseEstimator() {
        int[] tagCount = new int[cameras.length];
        double[] tagArea = new double[cameras.length];

        List<Integer> camerasToUpdate = new ArrayList<>();

        if (Robot.gameMode == GameMode.DISABLED) {
            for (int i = 0; i < cameras.length; i++) {
                boolean updateCamera = cameraHasTarget(i);
                // Logger.recordOutput("Subsystems/Vision/UpdateCamera" + i, updateCamera);
                if (updateCamera) {
                    camerasToUpdate.add(i);
                    tagCount[i] = inputs[i].tagIds.length;
                    tagArea[i] = (tagArea[i] * (camerasToUpdate.size() - 1) + inputs[i].averageTA) / camerasToUpdate.size();
                }
            }
        }
        else {
            camerasToUpdate.add(this.chooseCamera());
        }

        
        Logger.recordOutput("Subsystems/Vision/TagCount", tagCount);
        Logger.recordOutput("Subsystems/Vision/TagArea", tagArea);

        double xyStds = 0.0;
        double radStds = 0.0;

        int maxTagCount = 0;
        int maxTagIndex = 0;
        double maxTagArea = 0;

        // ArrayList<Integer> maxTagCams = new ArrayList<Integer>(); 

        if (camerasToUpdate.size() > 0) {
            // first find the greatest tag count 
            // create a new list of cameras with that many tags
            // then go through that list and use cameras that have the greates tag area over the rest

            // camera filtering
            for (int i = 0; i < camerasToUpdate.size(); i++) {
                if (inputs[camerasToUpdate.get(i)].tagIds.length > maxTagCount) { // find the max tag count
                    maxTagCount = inputs[camerasToUpdate.get(i)].tagIds.length;
                }
                if (inputs[camerasToUpdate.get(i)].averageTA > maxTagArea) { // find the max tag area
                    maxTagArea = inputs[camerasToUpdate.get(i)].averageTA;
                }
            }

            // for (int i = 0; i < camerasToUpdate.size(); i++) {
            //     if (inputs[camerasToUpdate.get(i)].tagIds.length > maxTagCount) { // find the max tag count
            //         maxTagCams.add(camerasToUpdate.get(i));
            //     }
            // }

            // for (int cam : maxTagCams) {
            //     if (camerasToUpdate.get(cam) > maxTagArea) {
            //         maxAreaIndex = maxTagCams.get(cam);
            //     }
            // }

            if (maxTagCount > 2 && maxTagArea > 0.10) {
                xyStds = Math.hypot(0.001, 0.002);
                radStds = 0.002;
            }
            else if (maxTagCount > 1 && maxTagArea > 0.10) {
                // Multiple targets or one huge target 
                // Trust the vision more
                xyStds = Math.hypot(0.01, 0.01);
                radStds = 0.015;
            }
            // 1 target with large area
            else if (maxTagArea > 0.18) {
                xyStds = Math.hypot(0.02, 0.02);
                radStds = 0.02;
            } else {
                return;
            }
        }

            // update the pose estimator
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, radStds));
            for (int cam : camerasToUpdate) {
                poseEstimator.addVisionMeasurement(inputs[cam].robotPose, inputs[cam].timestampSeconds);
            }
            // poseEstimator.addVisionMeasurement(inputs[maxAreaIndex].robotPose, inputs[maxAreaIndex].timestampSeconds);


        // cheese
        // if (camerasToUpdate.size() > 0) {
        //     for (int i = 0; i < camerasToUpdate.size(); i++) {
        //         if (tagCount[camerasToUpdate.get(i)] > maxTagCount) {
        //             maxTagCount = tagCount[camerasToUpdate.get(i)];
        //         } 
        //         if (tagArea[camerasToUpdate.get(i)] > maxTagArea) {
        //             maxTagArea = tagArea[camerasToUpdate.get(i)];
        //             maxTagIndex = i;
        //         }
        //         if (tagCount[camerasToUpdate.get(i)] >= 2) {
        //             twoTagIndex.add(camerasToUpdate.get(i));
        //         }
        //     }
        //     if (maxTagCount > 1 && maxTagArea > 0.08) {
        //         // Multiple targets or one huge target (like reef tag)
        //         // Trust the vision even MORE
        //         xyStds = Math.hypot(0.005, 0.005);
        //         radStds = 0.007;
        //     }
        //     // 1 target with large area
        //     else if (maxTagArea > 0.14) {
        //         xyStds = Math.hypot(0.009, 0.009);
        //         radStds = 0.01;
        //     } else {
        //         return;
        //     }
        //     // Conditions don't match to add a vision measurement
        // }

        // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, radStds));
        // if (maxTagCount > 1) {
        //     for (int i = 0; i < twoTagIndex.size(); i++) {
        //         //if (tagArea[twoTagIndex.get(i)] > 0.15) {
        //             poseEstimator.addVisionMeasurement(inputs[twoTagIndex.get(i)].robotPose, inputs[twoTagIndex.get(i)].timestampSeconds);
        //         //}
        //         System.out.println("Update camera: " + twoTagIndex.get(i));
        //     }
        // } 
        // if (maxTagCount == 1) {
        //     poseEstimator.addVisionMeasurement(inputs[camerasToUpdate.get(maxTagIndex)].robotPose, inputs[camerasToUpdate.get(maxTagIndex)].timestampSeconds);
        //     System.out.println("Update camera: " + camerasToUpdate.get(maxTagIndex));
        // }
        // if (!camerasToUpdate.isEmpty()) {
        //     rotationUpdated = true;
        // }
        // System.out.println("Max tag count: " + maxTagCount);

        // Logger.recordOutput("Subsystems/Vision/XYStdDev", xyStds);
        // Logger.recordOutput("Subsystems/Vision/ThetaStdDev", radStds);
    }

    private boolean cameraHasTarget(int cameraIndex) {
        if (!inputs[cameraIndex].robotPoseValid
            || Double.isNaN(inputs[cameraIndex].robotPose.getX()) 
            || Double.isNaN(inputs[cameraIndex].robotPose.getY()) 
            || Double.isNaN(inputs[cameraIndex].robotPose.getRotation().getRadians())
            || inputs[cameraIndex].robotPose.equals(Pose2d.kZero)
            || (shouldUseMT1() && inputs[cameraIndex].tagIds.length == 1 && inputs[cameraIndex].averageTA < 0.17))
        {
            return false;
        }
        return true;
    }


    public Command toggleMT1Command() {
        return runOnce(() -> this.useMT1Override = !this.useMT1Override);
    }

    public Command toggleEnabledThrottleCommand() {
        return runOnce(() -> this.enabledThrottleOverride = !this.enabledThrottleOverride);
    }

    public Command toggleDisableFour() {
        return runOnce(() -> this.disableFour = !this.disableFour);
    }

    @AutoLogOutput (key = "Subsystems/Vision/MT1")
    private boolean shouldUseMT1() {
        return (!(Robot.exitAuto)) && OIConstants.DRIVER_MODE != DriverMode.CALIBRATION && (Robot.gameMode == GameMode.DISABLED || !rotationUpdated) || useMT1Override;
    }

}
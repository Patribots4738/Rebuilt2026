package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Comparator;

import frc.robot.util.hardware.limelight.Limelight;
import frc.robot.util.hardware.limelight.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    
    private final Limelight camera;

    public VisionIOLimelight(String name, boolean isLL4) {
        this.camera = new Limelight(name, isLL4);
    }

    public void updateInputs(VisionIOInputs inputs) {
        camera.refreshPoseEstimate();
        inputs.robotPoseValid = camera.hasValidPoseEstimate();
        inputs.robotPose = camera.getRobotPose();
        inputs.averageTA = camera.getAverageTA();
        inputs.averageTD = camera.getAverageTD();
        inputs.timestampSeconds = camera.getTimestamp();
        RawFiducial[] rawFrontFiducials = camera.getRawFiducials();
        inputs.tagIds = new int[rawFrontFiducials.length];
        for (int i = 0; i < rawFrontFiducials.length; i++) {
            RawFiducial fid = rawFrontFiducials[i];
            inputs.tagIds[i] = fid.id;
        }
    }

    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }

    @Override
    public void setThrottle(int throttle) {
        camera.setThrottle(throttle);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        camera.setRobotOrientation(yawDegrees);
    }

    //depends on which lime
    @Override 
    public void setUseMegaTag2(boolean useMT2) {
        boolean isLL4 = camera.getIsLL4();
        if (useMT2) {
            if (isLL4) {
                camera.setIMUMode(2);
            }
        } else {
            if (isLL4) {
                camera.setIMUMode(1);
            } else {
                camera.setIMUMode(0);
            }
        }
        camera.setUseMT2(useMT2);
    }

    @Override
    public void setUsedTags(int[] tagIds) {
        camera.setValidTags(tagIds);
    }

    @Override
    public double getAverageTagDistance(){
        double totalTX = 0;
        double totalTY = 0;
        RawFiducial[] tags = camera.getRawFiducials();

        if (tags == null || tags.length < 2){
            totalTX += camera.getTX();
            totalTY += camera.getTY();
        }else{
            Arrays.sort(tags, Comparator.comparingDouble(tag -> Math.abs(tag.txnc)));
            double tx1 = tags[0].txnc;
            double tx2 = tags[1].txnc;
            Arrays.sort(tags, Comparator.comparingDouble(tag -> Math.abs(tag.tync)));
            double ty1 = tags[0].tync;
            double ty2 =  tags[1].tync;

            totalTX += (tx1 + tx2);
            totalTY += (ty1 + ty2);
        }

        return Math.sqrt(((Math.pow(totalTX, 2))+(Math.pow(totalTY, 2))));
    }


}
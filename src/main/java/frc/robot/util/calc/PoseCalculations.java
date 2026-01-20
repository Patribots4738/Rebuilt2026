package frc.robot.util.calc;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.util.Constants.FieldConstants;

public class PoseCalculations {

    public static Translation2d flipTranslationX(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), pos.getY());
    }

    public static Translation2d flipTranslationY(Translation2d pos) {
        return new Translation2d(pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    public static Translation2d mirrorTranslation(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }

    public static Rotation2d mirrorFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).plus(rotation);
    }

    public static Pose2d flipPoseX(Pose2d pos) {
        return new Pose2d(flipTranslationX(pos.getTranslation()), flipFieldRotation(pos.getRotation()));
    }

    public static Pose2d flipPoseY(Pose2d pos) {
        return new Pose2d(flipTranslationY(pos.getTranslation()), pos.getRotation());
    }

    public static Pose2d mirrorPose(Pose2d pos) {
        return new Pose2d(mirrorTranslation(pos.getTranslation()), mirrorFieldRotation(pos.getRotation()));
    }

    public static Pose3d flipPose3dX(Pose3d pos) {
        Translation2d trans = ((pos.getTranslation()).toTranslation2d());
        Rotation2d rotaZ = flipFieldRotation(pos.getRotation().toRotation2d());
        Rotation3d rota = new Rotation3d(pos.getRotation().getX(), pos.getRotation().getY(), rotaZ.getRadians());
        Pose3d newPos = new Pose3d(flipTranslationX(trans).getX(), pos.getY(), pos.getZ(), rota);
        return newPos;
    }

    public static Pose3d flipPose3dY(Pose3d pos) {
        Translation2d trans = ((pos.getTranslation()).toTranslation2d());
        Pose3d newPos = new Pose3d(pos.getX(), flipTranslationY(trans).getY(), pos.getZ(), pos.getRotation());
        return newPos;
    }

    public static Pose3d mirrorPose3d(Pose3d pos) {
        Pose3d newPos = new Pose3d(flipPose3dX(pos).getX(), flipPose3dY(pos).getY(), pos.getZ(), flipPose3dX(pos).getRotation());
        return newPos;
    }

    public static Pose2d getTrenchDimension(Pose3d pos, char side) {
        Pose2d newPos = new Pose2d();
        switch(side) {
            case 'L':
                newPos = new Pose2d((pos.getX() - FieldConstants.TRENCH_LENGTH), pos.getY(), pos.getRotation().toRotation2d());
            break;
            case 'R':
                newPos = new Pose2d((pos.getX() + FieldConstants.TRENCH_LENGTH), pos.getY(), pos.getRotation().toRotation2d());
            break;
            case 'U':
                newPos = new Pose2d(pos.getX(), (pos.getY() + FieldConstants.TRENCH_WIDTH), pos.getRotation().toRotation2d());
            break;
            case 'D':
                newPos = new Pose2d(pos.getX(), (pos.getY() - FieldConstants.TRENCH_WIDTH), pos.getRotation().toRotation2d());
            break;
        }
        return newPos;
    }

    public static Pose2d getBumpDimension(Pose3d pos, char side) {
        Pose2d newPos = new Pose2d();
        switch(side) {
            case 'L':
                newPos = new Pose2d((pos.getX() - FieldConstants.BUMP_LENGTH), pos.getY(), pos.getRotation().toRotation2d());
            break;
            case 'R':
                newPos = new Pose2d((pos.getX() + FieldConstants.BUMP_LENGTH), pos.getY(), pos.getRotation().toRotation2d());
            break;
            case 'U':
                newPos = new Pose2d(pos.getX(), (pos.getY() + FieldConstants.BUMP_WIDTH), pos.getRotation().toRotation2d());
            break;
            case 'D':
                newPos = new Pose2d(pos.getX(), (pos.getY() - FieldConstants.BUMP_WIDTH), pos.getRotation().toRotation2d());
            break;
        }
        return newPos;
    }

    public static Pose2d getTowerDimensions(Pose2d pos, String side) {
        Pose2d newPos = new Pose2d();
        switch(side) {
            case "PL":
                newPos = new Pose2d((pos.getX()), (pos.getY() - (FieldConstants.TOWER_POST_DISTANCE/2)), pos.getRotation());
            break;
            case "PR":
                newPos = new Pose2d((pos.getX()), (pos.getY() + (FieldConstants.TOWER_POST_DISTANCE/2)), pos.getRotation());
            break;
            case "SL":
                newPos = new Pose2d((pos.getX()), (pos.getY() - (FieldConstants.TOWER_WIDTH/2)), pos.getRotation());
            break;
            case "SR":
                newPos = new Pose2d((pos.getX()), (pos.getY() + (FieldConstants.TOWER_WIDTH/2)), pos.getRotation());
            break;
        }
        return newPos;
    }
    
}

package frc.robot.util.calc;

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
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;

public class PoseCalculations {

    public static Translation2d flipTranslationX(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), pos.getY());
    }

    public static Translation2d flipTranslationY(Translation2d pos) {
        return new Translation2d(pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    public static Translation2d mirrorTranslation2d(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    public static Translation3d flipTranslation3dX(Translation2d pos) {
        return new Translation3d(FieldConstants.FIELD_LENGTH - pos.getX(), pos.getY(), 0);
    }

    public static Translation3d flipTranslation3dY(Translation3d pos) {
        return new Translation3d(pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY(), 0);
    }


    public static Translation3d mirrorTranslation3d(Translation3d pos) {
        return new Translation3d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY(), pos.getZ());
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
        return new Pose2d(mirrorTranslation2d(pos.getTranslation()), mirrorFieldRotation(pos.getRotation()));
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

    

    public enum ZONE {
        inAllianceZone,
        inNeutralZone,
        inOppZone
    }

    public static ZONE getZone(Pose2d pos) {
        ZONE currentZone = ZONE.inAllianceZone;
        if (pos.getX() < FieldConstants.FREND_LINE()) {
            currentZone = ZONE.inAllianceZone;
        } else if (pos.getX() > FieldConstants.FREND_LINE() && pos.getX() < FieldConstants.FOW_LINE()) {
            currentZone = ZONE.inNeutralZone;
        } else if (pos.getX() > FieldConstants.FOW_LINE()) {
            currentZone = ZONE.inOppZone;
        }
        return currentZone;
    }

    public static Pose2d getPoseWithDistance(Pose2d pos, double distance) {
        return new Pose2d(pos.getX() + distance * pos.getRotation().getCos(), pos.getY() + distance * pos.getRotation().getSin(), pos.getRotation());
    }
    
    public static boolean isPoseNear(Pose2d pos1, Pose2d pos2) {
        double angleDiff = pos1.getRotation().minus(pos2.getRotation()).getRadians();
		double distance = pos1.relativeTo(pos2).getTranslation().getNorm();
        return 
            MathUtil.isNear(0, distance, AutoConstants.HDC_POSITION_TOLERANCE_METERS)
            && MathUtil.isNear(0, angleDiff, AutoConstants.HDC_ROTATION_TOLERANCE_RADIANS);
    }

    public static boolean isPoseOnAxis(Pose2d pos1, Pose2d pos2) {
        double distance = pos1.getTranslation().getDistance(pos2.getTranslation());
        Pose2d addedAxisPose = getPoseWithDistance(new Pose2d(pos1.getX(), pos1.getY(), pos2.getRotation().minus(Rotation2d.fromRadians(Math.PI))), distance);
        addedAxisPose = new Pose2d(addedAxisPose.getX(), addedAxisPose.getY(), pos2.getRotation());
        return isPoseNear(addedAxisPose, pos2);
    }

    public static Pose2d nearest(Pose2d pos, List<Pose2d> poses) {
        return Collections.min(poses,
            Comparator.comparing(
                    (Pose2d other) -> pos.getTranslation().getDistance(other.getTranslation()))
                .thenComparing(
                    (Pose2d other) ->
                        Math.abs(pos.getRotation().minus(other.getRotation()).getRadians())));
    }

    public static Rotation2d nearestRotation(Rotation2d pos, List<Rotation2d> poses) {
        return Collections.min(poses, 
            Comparator.comparing((Rotation2d other) -> Math.abs(pos.minus(other).getRadians())));
    }

    public static Pose3d nearestPose3d(Pose3d pos, List<Pose3d> poses) {
        return Collections.min(poses,
            Comparator.comparing(
                    (Pose3d other) -> pos.getTranslation().getDistance(other.getTranslation()))
                .thenComparing(
                    (Pose3d other) ->
                        Math.abs(pos.getRotation().toRotation2d().minus(other.getRotation().toRotation2d()).getRadians())));
    }

    public static Pose2d getClosestPost(Pose2d pos) {
        return nearest(pos, FieldConstants.GET_TOWER_POSTS());
    }

    public static Rotation2d getClosestBumpRotation(Pose2d pos) {
        return nearestRotation(pos.getRotation(), AutoConstants.BUMP_ROTATIONS);
    }

    public static boolean inTrench(Pose2d pos) {
        boolean ans = false;
        if (
            (pos.getY() > FieldConstants.TRENCHS.get(3).getY() && pos.getY() < FieldConstants.TRENCHS.get(9).getY()) 
            && 
            (
                (pos.getX() > FieldConstants.TRENCHS.get(1).getX() && pos.getX() < FieldConstants.TRENCHS.get(2).getX()) 
                || 
                (pos.getX() > FieldConstants.TRENCHS.get(11).getX() && pos.getX() < FieldConstants.TRENCHS.get(2).getX())
            )
            ) {
            ans = true;
        }
        return ans;
    }

    public static boolean onRamp(Pose2d pos) {
        boolean ans = false;
        if (
            (pos.getY() > FieldConstants.BUMPS.get(3).getY() && pos.getY() < FieldConstants.BUMPS.get(9).getY()) 
            && 
            (
                (pos.getX() > FieldConstants.BUMPS.get(1).getX() && pos.getX() < FieldConstants.BUMPS.get(2).getX()) 
                || 
                (pos.getX() > FieldConstants.BUMPS.get(11).getX() && pos.getX() < FieldConstants.BUMPS.get(2).getX())
            )
            ) {
            ans = true;
        }
        return ans;
    }

    public static boolean behindTower(Pose2d pos) {
        boolean ans = false;
        if (pos.getX() < FieldConstants.GET_TOWER_CENTER().getX()) {
            ans = true;
        }
        return ans;
    }
}

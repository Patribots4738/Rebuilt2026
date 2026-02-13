/**
 * Utility class containing static helper methods for performing
 * geometric calculations and field-relative transformations
 * for robot positioning.
 *
 * Includes translation flipping, pose mirroring, zone detection,
 * proximity checks, and field-element alignment helpers.
 */
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

    /**
     * Flips a 2D translation across the field length (X-axis reflection).
     *
     * @param pos original translation
     * @return flipped translation across field length
     */
    public static Translation2d flipTranslationX(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), pos.getY());
    }

    /**
     * Flips a 2D translation across the field width (Y-axis reflection).
     *
     * @param pos original translation
     * @return flipped translation across field width
     */
    public static Translation2d flipTranslationY(Translation2d pos) {
        return new Translation2d(pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    /**
     * Mirrors a translation across both field length and width.
     *
     * @param pos original translation
     * @return mirrored translation
     */
    public static Translation2d mirrorTranslation2d(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    /**
     * Flips a 2D translation across the field length and converts to 3D.
     *
     * @param pos original 2D translation
     * @return flipped 3D translation (Z = 0)
     */
    public static Translation3d flipTranslation3dX(Translation2d pos) {
        return new Translation3d(FieldConstants.FIELD_LENGTH - pos.getX(), pos.getY(), 0);
    }

    
    /**
     * Flips a 3D translation across the field width.
     *
     * @param pos original 3D translation
     * @return flipped 3D translation
     */
    public static Translation3d flipTranslation3dY(Translation3d pos) {
        return new Translation3d(pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY(), 0);
    }


    /**
     * Mirrors a 3D translation across both field axes.
     *
     * @param pos original 3D translation
     * @return mirrored 3D translation
     */
    public static Translation3d mirrorTranslation3d(Translation3d pos) {
        return new Translation3d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY(), pos.getZ());
    }

    /**
     * Flips a rotation across the field (180 degrees - rotation).
     *
     * @param rotation original rotation
     * @return flipped rotation
     */
    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }

    /**
     * Mirrors a rotation across the field (180° + rotation).
     *
     * @param rotation original rotation
     * @return mirrored rotation
     */
    public static Rotation2d mirrorFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).plus(rotation);
    }

    /**
     * Flips a Pose2d across the field length.
     *
     * @param pos original pose
     * @return flipped pose
     */
    public static Pose2d flipPoseX(Pose2d pos) {
        return new Pose2d(flipTranslationX(pos.getTranslation()), flipFieldRotation(pos.getRotation()));
    }

    /**
     * Flips a Pose2d across the field width.
     *
     * @param pos original pose
     * @return flipped pose
     */
    public static Pose2d flipPoseY(Pose2d pos) {
        return new Pose2d(flipTranslationY(pos.getTranslation()), pos.getRotation());
    }


    /**
     * Mirrors a Pose2d across both field axes.
     *
     * @param pos original pose
     * @return mirrored pose
     */
    public static Pose2d mirrorPose(Pose2d pos) {
        return new Pose2d(mirrorTranslation2d(pos.getTranslation()), mirrorFieldRotation(pos.getRotation()));
    }

    /**
     * Flips a 3D pose across the field length.
     *
     * @param pos original 3D pose
     * @return flipped 3D pose
     */
    public static Pose3d flipPose3dX(Pose3d pos) {
        Translation2d trans = ((pos.getTranslation()).toTranslation2d());
        Rotation2d rotaZ = flipFieldRotation(pos.getRotation().toRotation2d());
        Rotation3d rota = new Rotation3d(pos.getRotation().getX(), pos.getRotation().getY(), rotaZ.getRadians());
        Pose3d newPos = new Pose3d(flipTranslationX(trans).getX(), pos.getY(), pos.getZ(), rota);
        return newPos;
    }


    /**
     * Flips a 3D pose across the field width.
     *
     * @param pos original 3D pose
     * @return flipped 3D pose
     */
    public static Pose3d flipPose3dY(Pose3d pos) {
        Translation2d trans = ((pos.getTranslation()).toTranslation2d());
        Pose3d newPos = new Pose3d(pos.getX(), flipTranslationY(trans).getY(), pos.getZ(), pos.getRotation());
        return newPos;
    }

    
    /**
     * Mirrors a 3D pose across both field axes.
     *
     * @param pos original 3D pose
     * @return mirrored 3D pose
     */
    public static Pose3d mirrorPose3d(Pose3d pos) {
        Pose3d newPos = new Pose3d(flipPose3dX(pos).getX(), flipPose3dY(pos).getY(), pos.getZ(), flipPose3dX(pos).getRotation());
        return newPos;
    }


    /**
     * Offsets a 3D pose by trench dimensions depending on side.
     *
     * @param pos 3D reference pose
     * @param side L, R, U, or D for direction
     * @return adjusted Pose2d
     */
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


    /**
     * Offsets a 3D pose by bump dimensions depending on side.
     *
     * @param pos 3D reference pose
     * @param side L, R, U, or D for direction
     * @return adjusted Pose2d
     */
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

    /**
     * Returns a pose adjusted to a specific tower reference location.
     *
     * @param pos base pose
     * @param side PL, PR, SL, or SR (post left/right, side left/right)
     * @return adjusted pose
     */
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

    /*
     * Lists representing field zones
     */
    

    public enum ZONE {
        /** Robot is in alliance zone */
        inAllianceZone,
        /** Robot is in neutral zone */
        inNeutralZone,
        /** Robot is in opponent zone */
        inOppZone
    }


    /**
     * Determines which field zone the pose is in.
     *
     * @param pos robot pose
     * @return current zone
     */
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

    /**
     * Returns a new pose translated forward along its rotation.
     *
     * @param pos original pose
     * @param distance forward distance in meters
     * @return new pose with applied offset
     */
    public static Pose2d getPoseWithDistance(Pose2d pos, double distance) {
        return new Pose2d(pos.getX() + distance * pos.getRotation().getCos(), pos.getY() + distance * pos.getRotation().getSin(), pos.getRotation());
    }
    

    /**
     * Determines whether two poses are within configured
     * position and rotation tolerances.
     *
     * @param pos1 first pose
     * @param pos2 second pose
     * @return true if poses are near each other
     */
    public static boolean isPoseNear(Pose2d pos1, Pose2d pos2) {
        double angleDiff = pos1.getRotation().minus(pos2.getRotation()).getRadians();
		double distance = pos1.relativeTo(pos2).getTranslation().getNorm();
        return 
            MathUtil.isNear(0, distance, AutoConstants.HDC_POSITION_TOLERANCE_METERS)
            && MathUtil.isNear(0, angleDiff, AutoConstants.HDC_ROTATION_TOLERANCE_RADIANS);
    }


    /**
     * Checks whether pos1 lies on the rotational axis of pos2.
     *
     * @param pos1 first pose
     * @param pos2 reference pose
     * @return true if pos1 is aligned on pos2 axis
     */
    public static boolean isPoseOnAxis(Pose2d pos1, Pose2d pos2) {
        double distance = pos1.getTranslation().getDistance(pos2.getTranslation());
        Pose2d addedAxisPose = getPoseWithDistance(new Pose2d(pos1.getX(), pos1.getY(), pos2.getRotation().minus(Rotation2d.fromRadians(Math.PI))), distance);
        addedAxisPose = new Pose2d(addedAxisPose.getX(), addedAxisPose.getY(), pos2.getRotation());
        return isPoseNear(addedAxisPose, pos2);
    }


    /**
     * Returns the nearest pose from a list based on
     * translation distance and rotation difference.
     *
     * @param pos reference pose
     * @param poses list of candidate poses
     * @return closest pose
     */
    public static Pose2d nearest(Pose2d pos, List<Pose2d> poses) {
        return Collections.min(poses,
            Comparator.comparing(
                    (Pose2d other) -> pos.getTranslation().getDistance(other.getTranslation()))
                .thenComparing(
                    (Pose2d other) ->
                        Math.abs(pos.getRotation().minus(other.getRotation()).getRadians())));
    }


    /**
     * Returns the nearest rotation from a list.
     *
     * @param pos reference rotation
     * @param poses list of candidate rotations
     * @return closest rotation
     */
    public static Rotation2d nearestRotation(Rotation2d pos, List<Rotation2d> poses) {
        return Collections.min(poses, 
            Comparator.comparing((Rotation2d other) -> Math.abs(pos.minus(other).getRadians())));
    }


    /**
     * Returns the nearest 3D pose from a list.
     *
     * @param pos reference pose
     * @param poses list of candidate 3D poses
     * @return closest 3D pose
     */
    public static Pose3d nearestPose3d(Pose3d pos, List<Pose3d> poses) {
        return Collections.min(poses,
            Comparator.comparing(
                    (Pose3d other) -> pos.getTranslation().getDistance(other.getTranslation()))
                .thenComparing(
                    (Pose3d other) ->
                        Math.abs(pos.getRotation().toRotation2d().minus(other.getRotation().toRotation2d()).getRadians())));
    }

    /**
     * Returns the closest tower post to the given pose.
     *
     * @param pos robot pose
     * @return nearest tower post pose
     */
    public static Pose2d getClosestPost(Pose2d pos) {
        return nearest(pos, FieldConstants.GET_TOWER_POSTS());
    }

    /**
     * Returns the closest bump rotation to the pose's rotation.
     *
     * @param pos robot pose
     * @return closest bump rotation
     */
    public static Rotation2d getClosestBumpRotation(Pose2d pos) {
        return nearestRotation(pos.getRotation(), AutoConstants.BUMP_ROTATIONS);
    }


    /**
     * Determines whether a pose lies within the trench region.
     *
     * @param pos robot pose
     * @return true if inside trench
     */
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

    /**
     * Forward offset of the shooter relative to the robot origin (meters).
     */
    public static final double SHOOTER_FORWARD_OFFSET_METERS = 0.0;

    
    /**
     * Determines whether the hood should be lowered based on
     * whether the shooter (not robot center) is inside the trench.
     *
     * @param pos robot pose
     * @return true if hood should be lowered
     */
    public static boolean shouldPutHoodDown(Pose2d pos) {
        Pose2d shooterPose = getPoseWithDistance(pos, SHOOTER_FORWARD_OFFSET_METERS);
        return inTrench(shooterPose);
    }


    /**
     * Determines whether the pose is on the ramp (bump region).
     *
     * @param pos robot pose
     * @return true if on ramp
     */
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


    /**
     * Determines whether the robot is behind the tower center.
     *
     * @param pos robot pose
     * @return true if behind tower
     */
    public static boolean behindTower(Pose2d pos) {
        boolean ans = false;
        if (pos.getX() < FieldConstants.GET_TOWER_CENTER().getX()) {
            ans = true;
        }
        return ans;
    }
}

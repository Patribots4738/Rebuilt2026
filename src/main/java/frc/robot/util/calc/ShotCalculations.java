package frc.robot.util.calc;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.TurretConstants;


public class ShotCalculations {

    private final static LinearFilter tAngleFilter = LinearFilter.movingAverage((int) (0.1 / 33.3333));
    private final static LinearFilter hAngleFilter = LinearFilter.movingAverage((int) (0.1 / 33.3333));



    private static Rotation2d lastTurretAngle;
    private static double lastHoodAngleRadians;
    private static Rotation2d turretAngle;
    private static double hoodAngleRadians = Double.NaN;
    private static double turretVelocity;
    private static double hoodVelocity;

    private static double minDistance;
    private static double maxDistance;

    private static final InterpolatingTreeMap<Double, Rotation2d> hubShotAngleHoodMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingTreeMap<Double, Rotation2d> passingShotAngleHoodMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private static final InterpolatingDoubleTreeMap hubShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    public static record ShootingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed) 
    {}

    private static ShootingParameters latestParameters = null;

    static {

        //TODO: tune on our robot
        minDistance = 1.34;
        maxDistance = 5.60;

        //Hub
        hubShotAngleHoodMap.put(1.34, Rotation2d.fromDegrees(19.0));
        hubShotAngleHoodMap.put(1.78, Rotation2d.fromDegrees(19.0));
        hubShotAngleHoodMap.put(2.17, Rotation2d.fromDegrees(24.0));
        hubShotAngleHoodMap.put(2.81, Rotation2d.fromDegrees(27.0));
        hubShotAngleHoodMap.put(3.82, Rotation2d.fromDegrees(29.0));
        hubShotAngleHoodMap.put(4.09, Rotation2d.fromDegrees(30.0));
        hubShotAngleHoodMap.put(4.40, Rotation2d.fromDegrees(31.0));
        hubShotAngleHoodMap.put(4.77, Rotation2d.fromDegrees(32.0));
        hubShotAngleHoodMap.put(5.57, Rotation2d.fromDegrees(32.0));
        hubShotAngleHoodMap.put(5.60, Rotation2d.fromDegrees(35.0));

        hubShotFlywheelSpeedMap.put(1.34, 210.0);
        hubShotFlywheelSpeedMap.put(1.78, 220.0);
        hubShotFlywheelSpeedMap.put(2.17, 220.0);
        hubShotFlywheelSpeedMap.put(2.81, 230.0);
        hubShotFlywheelSpeedMap.put(3.82, 250.0);
        hubShotFlywheelSpeedMap.put(4.09, 255.0);
        hubShotFlywheelSpeedMap.put(4.40, 260.0);
        hubShotFlywheelSpeedMap.put(4.77, 265.0);
        hubShotFlywheelSpeedMap.put(5.57, 275.0);
        hubShotFlywheelSpeedMap.put(5.60, 290.0);

        hubTimeOfFlightMap.put(5.68, 1.16);
        hubTimeOfFlightMap.put(4.55, 1.12);
        hubTimeOfFlightMap.put(3.15, 1.11);
        hubTimeOfFlightMap.put(1.88, 1.09);
        hubTimeOfFlightMap.put(1.38, 0.90);

        //Passing
        passingShotAngleHoodMap.put(1.34, Rotation2d.fromDegrees(19.0));
        passingShotAngleHoodMap.put(1.78, Rotation2d.fromDegrees(19.0));
        passingShotAngleHoodMap.put(2.17, Rotation2d.fromDegrees(24.0));
        passingShotAngleHoodMap.put(2.81, Rotation2d.fromDegrees(27.0));
        passingShotAngleHoodMap.put(3.82, Rotation2d.fromDegrees(29.0));
        passingShotAngleHoodMap.put(4.09, Rotation2d.fromDegrees(30.0));
        passingShotAngleHoodMap.put(4.40, Rotation2d.fromDegrees(31.0));
        passingShotAngleHoodMap.put(4.77, Rotation2d.fromDegrees(32.0));
        passingShotAngleHoodMap.put(5.57, Rotation2d.fromDegrees(32.0));
        passingShotAngleHoodMap.put(5.60, Rotation2d.fromDegrees(35.0));

        passingShotFlywheelSpeedMap.put(1.34, 210.0);
        passingShotFlywheelSpeedMap.put(1.78, 220.0);
        passingShotFlywheelSpeedMap.put(2.17, 220.0);
        passingShotFlywheelSpeedMap.put(2.81, 230.0);
        passingShotFlywheelSpeedMap.put(3.82, 250.0);
        passingShotFlywheelSpeedMap.put(4.09, 255.0);
        passingShotFlywheelSpeedMap.put(4.40, 260.0);
        passingShotFlywheelSpeedMap.put(4.77, 265.0);
        passingShotFlywheelSpeedMap.put(5.57, 275.0);
        passingShotFlywheelSpeedMap.put(5.60, 290.0);

        passingTimeOfFlightMap.put(5.68, 1.16);
        passingTimeOfFlightMap.put(4.55, 1.12);
        passingTimeOfFlightMap.put(3.15, 1.11);
        passingTimeOfFlightMap.put(1.88, 1.09);
        passingTimeOfFlightMap.put(1.38, 0.90);
    }

    public static ShootingParameters getParameters(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotRelativeVelocitySupplier, Translation3d target, boolean hub) {
        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = robotPoseSupplier.get();
        ChassisSpeeds robotRelativeVelocity = robotRelativeVelocitySupplier.get();
        estimatedPose =
            estimatedPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * LoggingConstants.PHASE_DELAY,
                    robotRelativeVelocity.vyMetersPerSecond * LoggingConstants.PHASE_DELAY,
                    robotRelativeVelocity.omegaRadiansPerSecond * LoggingConstants.PHASE_DELAY));

        // Calculate distance from turret to target
        Pose2d turretPosition = estimatedPose.transformBy(TurretConstants.TURRET_TO_ROBOT_TRANSFORM2D);
        double turretToTargetDistance = target.toTranslation2d().getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = robotRelativeVelocitySupplier.get();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_TO_ROBOT_TRANSFORM3D.getY() * Math.cos(robotAngle)
                        - TurretConstants.TURRET_TO_ROBOT_TRANSFORM3D.getX() * Math.sin(robotAngle));
        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_TO_ROBOT_TRANSFORM3D.getX() * Math.cos(robotAngle)
                        - TurretConstants.TURRET_TO_ROBOT_TRANSFORM3D.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = hub ? hubTimeOfFlightMap.get(lookaheadTurretToTargetDistance) : passingTimeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.toTranslation2d().getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = target.toTranslation2d().minus(lookaheadPose.getTranslation()).getAngle();
        hoodAngleRadians = hub ? hubShotAngleHoodMap.get(lookaheadTurretToTargetDistance).getRadians() : passingShotAngleHoodMap.get(lookaheadTurretToTargetDistance).getRadians();
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngleRadians)) lastHoodAngleRadians = hoodAngleRadians;
        turretVelocity =
            tAngleFilter.calculate(
                turretAngle.minus(lastTurretAngle).getRadians() / 33.3333);
        hoodVelocity =
            hAngleFilter.calculate((hoodAngleRadians - lastHoodAngleRadians) / 33.3333);
        lastTurretAngle = turretAngle;
        lastHoodAngleRadians = hoodAngleRadians;
        latestParameters =
            new ShootingParameters(
                lookaheadTurretToTargetDistance >= minDistance
                    && lookaheadTurretToTargetDistance <= maxDistance,
                turretAngle,
                turretVelocity,
                hoodAngleRadians,
                hoodVelocity,
                hub ? 
                    hubShotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance) :
                    passingShotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)
            );

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestParameters;
    }
}
package frc.robot.util.calc;

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
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.TurretConstants;

public class ShotCalculations {

    private final LinearFilter tAngleFilter = LinearFilter.movingAverage((int) (0.1 / 33.3333));
    private final LinearFilter hAngleFilter = LinearFilter.movingAverage((int) (0.1 / 33.3333));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngleRadians;
    private Rotation2d turretAngle;
    private double hoodAngleRadians = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;

    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public record ShootingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed) 
    {}

    private ShootingParameters latestParameters = null;



    static {
        //TODO: tune on our robot
        minDistance = 1.34;
        maxDistance = 5.60;
        phaseDelay = 0.03;

        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

        shotFlywheelSpeedMap.put(1.34, 210.0);
        shotFlywheelSpeedMap.put(1.78, 220.0);
        shotFlywheelSpeedMap.put(2.17, 220.0);
        shotFlywheelSpeedMap.put(2.81, 230.0);
        shotFlywheelSpeedMap.put(3.82, 250.0);
        shotFlywheelSpeedMap.put(4.09, 255.0);
        shotFlywheelSpeedMap.put(4.40, 260.0);
        shotFlywheelSpeedMap.put(4.77, 265.0);
        shotFlywheelSpeedMap.put(5.57, 275.0);
        shotFlywheelSpeedMap.put(5.60, 290.0);

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }

    public ShootingParameters getParameters(Swerve swerve, Translation3d target) {
        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = swerve.getPose();
        ChassisSpeeds robotRelativeVelocity = swerve.getRobotRelativeVelocity();
        estimatedPose =
            estimatedPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Pose2d turretPosition = estimatedPose.transformBy(TurretConstants.TURRET_TO_ROBOT_TRANSFORM2D);
        double turretToTargetDistance = target.toTranslation2d().getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = swerve.getRobotRelativeVelocity();
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
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
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
        hoodAngleRadians = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
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
                shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
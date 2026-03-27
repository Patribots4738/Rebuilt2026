package frc.robot.util.calc;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.TurretConstants;


public class ShotCalculations {

    private final static LinearFilter tAngleFilter = LinearFilter.movingAverage((int) (0.1 / LoggingConstants.PHASE_DELAY));
    private final static LinearFilter hAngleFilter = LinearFilter.movingAverage((int) (0.1 / LoggingConstants.PHASE_DELAY));



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
        hubShotAngleHoodMap.put(1.03251, Rotation2d.fromRadians(0.035));
        hubShotAngleHoodMap.put(1.64211, Rotation2d.fromRadians(0.10472));
        hubShotAngleHoodMap.put(1.94691, Rotation2d.fromRadians(0.139626));
        hubShotAngleHoodMap.put(2.25171, Rotation2d.fromRadians(0.139626));
        hubShotAngleHoodMap.put(2.55651, Rotation2d.fromRadians(0.15708));
        hubShotAngleHoodMap.put(2.86131, Rotation2d.fromRadians(0.15708));
        hubShotAngleHoodMap.put(3.16611, Rotation2d.fromRadians(0.174533));
        hubShotAngleHoodMap.put(Units.inchesToMeters(136.65), Rotation2d.fromRadians(0.19186));
        hubShotAngleHoodMap.put(Units.inchesToMeters(148.65), Rotation2d.fromRadians(0.244346));
        hubShotAngleHoodMap.put(Units.inchesToMeters(160.65), Rotation2d.fromRadians(0.244346));
        hubShotAngleHoodMap.put(Units.inchesToMeters(172.65), Rotation2d.fromRadians(0.279253));
        hubShotAngleHoodMap.put(Units.inchesToMeters(184.65), Rotation2d.fromRadians(0.279253));
        hubShotAngleHoodMap.put(Units.inchesToMeters(196.65), Rotation2d.fromRadians(0.261799));
        hubShotAngleHoodMap.put(Units.inchesToMeters(208.65), Rotation2d.fromRadians(0.261799));
        hubShotAngleHoodMap.put(Units.inchesToMeters(220.65), Rotation2d.fromRadians(0.279253));
        hubShotAngleHoodMap.put(Units.inchesToMeters(232.65), Rotation2d.fromRadians(0.296706));
        hubShotAngleHoodMap.put(Units.inchesToMeters(244.65), Rotation2d.fromRadians(0.314159));
        hubShotAngleHoodMap.put(Units.inchesToMeters(256.65), Rotation2d.fromRadians(0.314159));
        hubShotAngleHoodMap.put(Units.inchesToMeters(268.65), Rotation2d.fromRadians(0.331613));
        hubShotAngleHoodMap.put(Units.inchesToMeters(280.65), Rotation2d.fromRadians(0.349066));
        hubShotAngleHoodMap.put(Units.inchesToMeters(292.65), Rotation2d.fromRadians(0.349066));

        hubShotFlywheelSpeedMap.put(1.03251, 3000.0);
        hubShotFlywheelSpeedMap.put(1.64211, 3000.0);
        hubShotFlywheelSpeedMap.put(1.94691, 3000.0);
        hubShotFlywheelSpeedMap.put(2.25171, 3200.0);
        hubShotFlywheelSpeedMap.put(2.55651, 3300.0);
        hubShotFlywheelSpeedMap.put(2.86131, 3400.0);
        hubShotFlywheelSpeedMap.put(3.16611, 3500.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(136.65), 3400.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(148.65), 3500.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(160.65), 3700.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(172.65), 3700.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(184.65), 3800.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(196.65), 3900.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(208.65), 4000.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(220.65), 4100.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(232.65), 4100.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(244.65), 4200.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(256.65), 4300.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(268.65), 4400.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(280.65), 4400.0);
        hubShotFlywheelSpeedMap.put(Units.inchesToMeters(292.65), 4500.0);

        hubTimeOfFlightMap.put(1.03251, 1.23);
        hubTimeOfFlightMap.put(1.64211, 0.95);
        hubTimeOfFlightMap.put(1.94691, 1.23);
        hubTimeOfFlightMap.put(2.25171, 1.35);
        hubTimeOfFlightMap.put(2.55651, 1.32);
        hubTimeOfFlightMap.put(2.86131, 1.4);
        hubTimeOfFlightMap.put(3.16611, 1.4);
        hubTimeOfFlightMap.put(Units.inchesToMeters(136.65), 1.44);
        hubTimeOfFlightMap.put(Units.inchesToMeters(148.65), 1.31);
        hubTimeOfFlightMap.put(Units.inchesToMeters(160.65), 1.35);
        hubTimeOfFlightMap.put(Units.inchesToMeters(172.65), 1.32);
        hubTimeOfFlightMap.put(Units.inchesToMeters(184.65), 1.34);
        hubTimeOfFlightMap.put(Units.inchesToMeters(196.65), 1.45);
        hubTimeOfFlightMap.put(Units.inchesToMeters(208.65), 1.5);
        hubTimeOfFlightMap.put(Units.inchesToMeters(220.65), 1.46);
        hubTimeOfFlightMap.put(Units.inchesToMeters(232.65), 1.55);
        hubTimeOfFlightMap.put(Units.inchesToMeters(244.65), 1.44);
        hubTimeOfFlightMap.put(Units.inchesToMeters(256.65), 1.49);
        hubTimeOfFlightMap.put(Units.inchesToMeters(268.65), 1.51);
        hubTimeOfFlightMap.put(Units.inchesToMeters(280.65), 1.5);
        hubTimeOfFlightMap.put(Units.inchesToMeters(292.65), 1.53);

        //Passing
        passingShotAngleHoodMap.put(1.03251, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(1.64211, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(2.20091, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(2.50571, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(2.81051, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(3.36931, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(3.67411, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(4.28371, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(4.84251, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(5.14731, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(5.45211, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(5.75691, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(6.06171, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(6.36651, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(6.67131, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(6.97611, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(7.28091, Rotation2d.fromRadians(0.314159));
        passingShotAngleHoodMap.put(7.58571, Rotation2d.fromRadians(0.314159));


        passingShotFlywheelSpeedMap.put(1.03251, 2600.0);
        passingShotFlywheelSpeedMap.put(1.64211, 2600.0);
        passingShotFlywheelSpeedMap.put(2.20091, 3000.0);
        passingShotFlywheelSpeedMap.put(2.50571, 3100.0);
        passingShotFlywheelSpeedMap.put(2.81051, 3100.0);
        passingShotFlywheelSpeedMap.put(3.36931, 3400.0);
        passingShotFlywheelSpeedMap.put(3.67411, 3400.0);
        passingShotFlywheelSpeedMap.put(4.28371, 3400.0);
        passingShotFlywheelSpeedMap.put(4.84251, 3600.0);
        passingShotFlywheelSpeedMap.put(5.14731, 3600.0);
        passingShotFlywheelSpeedMap.put(5.45211, 3700.0);
        passingShotFlywheelSpeedMap.put(5.75691, 3700.0);
        passingShotFlywheelSpeedMap.put(6.06171, 3900.0);
        passingShotFlywheelSpeedMap.put(6.36651, 3900.0);
        passingShotFlywheelSpeedMap.put(6.67131, 4000.0);
        passingShotFlywheelSpeedMap.put(6.97611, 4100.0);
        passingShotFlywheelSpeedMap.put(7.28091, 4200.0);
        passingShotFlywheelSpeedMap.put(7.58571, 4300.0);

        passingTimeOfFlightMap.put(1.03251, 1d);
        passingTimeOfFlightMap.put(1.64211, 0.9);
        passingTimeOfFlightMap.put(2.20091, 1.15);
        passingTimeOfFlightMap.put(2.50571, 1.2);
        passingTimeOfFlightMap.put(2.81051, 1.15);
        passingTimeOfFlightMap.put(3.36931, 1.35);
        passingTimeOfFlightMap.put(3.67411, 1.25);
        passingTimeOfFlightMap.put(4.28371, 1.25);
        passingTimeOfFlightMap.put(4.84251, 1.4);
        passingTimeOfFlightMap.put(5.14731, 1.35);
        passingTimeOfFlightMap.put(5.45211, 1.35);
        passingTimeOfFlightMap.put(5.75691, 1.35);
        passingTimeOfFlightMap.put(6.06171, 1.5);
        passingTimeOfFlightMap.put(6.36651, 1.3);
        passingTimeOfFlightMap.put(6.67131, 1.45);
        passingTimeOfFlightMap.put(6.97611, 1.45);
        passingTimeOfFlightMap.put(7.28091, 1.45);
        passingTimeOfFlightMap.put(7.58571, 1.45);
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
        turretAngle = target.toTranslation2d().minus(lookaheadPose.getTranslation()).getAngle().minus(robotPoseSupplier.get().getRotation());
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
        // Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        // Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
        // Logger.recordOutput("ShotCalculator/HoodAngleDegrees", Units.radiansToDegrees(latestParameters.hoodAngle));
        // Logger.recordOutput("ShotCalculator/TurretAngleDegrees", latestParameters.turretAngle.getDegrees());
        // Logger.recordOutput("ShotCalculator/FlywheelSpeedRPM", latestParameters.flywheelSpeed);

        return latestParameters;
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.custom.LoggedGainConstants;
import frc.robot.util.custom.LoggedTunableBoolean;
import frc.robot.util.custom.LoggedTunableConstant;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.rev.Neo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class LoggingConstants {
        
        private static RobotType robotType = RobotType.DEVBOT;

        public static RobotType getRobot() {
            if (!FieldConstants.IS_SIMULATION && robotType == RobotType.SIMBOT) {
                System.out.println("Incorrect robot type selected, changing to real robot");
                robotType = RobotType.COMPBOT;
            }
            return robotType;
        }

        public static Mode getMode() {
            return switch (getRobot()) {
                case DEVBOT -> FieldConstants.IS_SIMULATION ? Mode.SIM : Mode.REAL;
                case COMPBOT -> FieldConstants.IS_SIMULATION ? Mode.REPLAY : Mode.REAL;
                case SIMBOT -> Mode.SIM;
            };
        }

        public enum Mode {
            /** Running on a real robot. */
            REAL,
            /** Running a physics simulator. */
            SIM,
            /** Replaying from a log file. */
            REPLAY
        }

        public enum RobotType {
            DEVBOT,
            COMPBOT,
            SIMBOT
        }

        public static final int CLIMB_INDEX = 0;
        public static final int EXTENDER_INDEX = 1;
        public static final int TURRET_INDEX = 2;
        public static final int HOOD_INDEX = 3;

        public static final Translation3d ROBOT_OFFSET = new Translation3d(
            0.0,
            0.0,
            0.0
        );

        public static final Translation3d CLIMB_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() + 0.056,
            ROBOT_OFFSET.getY() + 0.241,
            ROBOT_OFFSET.getZ() + 0.2735
        );

        public static final Translation3d EXTENDER_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() + 0.0,
            ROBOT_OFFSET.getY() + 0.0,
            ROBOT_OFFSET.getZ() + 0.0
        );

        public static final Translation3d TURRET_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() - 4.5,
            ROBOT_OFFSET.getY() -4.0,
            ROBOT_OFFSET.getZ() + 12.594
        );

        public static final Translation3d HOOD_OFFSET = new Translation3d(
            ROBOT_OFFSET.getX() + -9.392,
            ROBOT_OFFSET.getY() + 0.0,
            ROBOT_OFFSET.getZ() + 17.156
        );

        public static final double PHASE_DELAY = 0.03;
        public static final double PHASE_PER_SECOND = 1.0 / PHASE_DELAY;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = 4.0;

        public static double DEFAULT_DRIVE_MULTIPLIER = 1.0;
        public static double SHOOTING_DRIVE_MULTIPLIER = 0.65;

        public static double SKID_THRESHOLD = 0.1;

        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Units.degreesToRadians(1137.21); // radians per second

        public static final double ODOMETRY_FREQUENCY = 250.0;

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.75);
        // Distance between front and back wheels on robot
        // Easiest measured from the center of the bore of the vortex
        public static final double WHEEL_BASE = Units.inchesToMeters(21.75);

        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(27);
        public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(2.75); //update

        public static final double FULL_ROBOT_LENGTH_METERS = ROBOT_LENGTH_METERS + (2*BUMPER_LENGTH_METERS);

        // Front positive, left positive
        public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        public static final Translation2d[] WHEEL_POSITION_ARRAY = new Translation2d[] {
            FRONT_LEFT_WHEEL_POSITION,
            FRONT_RIGHT_WHEEL_POSITION,
            REAR_LEFT_WHEEL_POSITION,
            REAR_RIGHT_WHEEL_POSITION
        };

        public static final SwerveModuleState[] X_WHEEL_STATES = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        public static final SwerveModuleState[] O_WHEEL_STATES = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        public static final ChassisSpeeds ZEROED_SPEEDS = new ChassisSpeeds();
        public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(DriveConstants.MAX_SPEED_METERS_PER_SECOND, DriveConstants.MAX_SPEED_METERS_PER_SECOND, DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);


        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                WHEEL_POSITION_ARRAY
        );

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90); // invert drive
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0); // good
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-180); // good
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90); // invert drive


        // public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180 + 90);
        // public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90 + 90);
        // public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90 + 90);
        // public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0 + 90);

        public static final int GYRO_CAN_ID = 29;
        public static final boolean GYRO_REVERSED = true;

        public static final int FRONT_LEFT_INDEX = 0;
        public static final int FRONT_RIGHT_INDEX = 1;
        public static final int REAR_LEFT_INDEX = 2;
        public static final int REAR_RIGHT_INDEX = 3;

    }

    public static final class AutoConstants {

        //TODO: tune these values
        public static final double HDC_XY_ACCELERATION = 2.5;
        public static final double HDC_XY_VELOCITY = 1;

        public static final double HDC_THETA_ACCELERATION =  Units.degreesToRadians(450d);
        public static final double HDC_THETA_VELOCITY =  Units.degreesToRadians(270d);

        public static final double HDC_POSITION_TOLERANCE_METERS = Units.inchesToMeters(2);
        public static final double HDC_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);

        public static final double REEF_ALIGNMENT_MAX_SPEED = 1.0;
        public static final double INTAKE_ALIGNMENT_MAX_SPEED = 1.0;
        public static final double PROCESSOR_ALIGNMENT_MAX_SPEED = 0.0;

        public static final double REEF_ALIGNMENT_PREP_DISTANCE = 1.0;


        public static final GainConstants AUTO_XY_GAINS = new GainConstants(
            1d,
            0.5,
            0.1
        );

        public static final GainConstants TELE_XY_GAINS = new GainConstants(
            3.25,
            0,
            0
        );

        // public static final LoggedGainConstants LOGGED_TELE_XY_GAINS = new LoggedGainConstants(AutoConstants.TELE_XY_GAINS, "TeleXY");

        public static final GainConstants AUTO_THETA_GAINS = new GainConstants(
            2.5,
            0,
            0
        );

        public static final GainConstants TELE_THETA_GAINS = new GainConstants(
            5.5,
            0,
            0
        );

        // public static final LoggedGainConstants LOGGED_TELE_THETA_GAINS = new LoggedGainConstants(AutoConstants.TELE_THETA_GAINS, "TeleTheta");

        public static final PIDController XY_PID = new PIDController(
            AutoConstants.TELE_XY_GAINS.getP(),
            AutoConstants.TELE_XY_GAINS.getI(),
            AutoConstants.TELE_XY_GAINS.getD()
        );


        public static final ProfiledPIDController THETA_PID = new ProfiledPIDController(
            AutoConstants.TELE_THETA_GAINS.getP(),
            AutoConstants.TELE_THETA_GAINS.getI(),
            AutoConstants.TELE_THETA_GAINS.getD(),
            new TrapezoidProfile.Constraints(
                AutoConstants.HDC_THETA_VELOCITY,
                AutoConstants.HDC_THETA_ACCELERATION))
            {{
                setIZone(Units.degreesToRadians(45));
            }};

        public static HolonomicDriveController TELE_HDC = new HolonomicDriveController(
            XY_PID,
            XY_PID,
            THETA_PID
        );

        public static PPHolonomicDriveController AUTO_HDC = new PPHolonomicDriveController(
            new PIDConstants(
                AutoConstants.AUTO_XY_GAINS.getP(),
                AutoConstants.AUTO_XY_GAINS.getI(),
                AutoConstants.AUTO_XY_GAINS.getD()),
            new PIDConstants(
                AutoConstants.AUTO_THETA_GAINS.getP(),
                AutoConstants.AUTO_THETA_GAINS.getI(),
                AutoConstants.AUTO_THETA_GAINS.getD(),
                Units.degreesToRadians(45)));

        public static final String[] AUTO_NAMES = new String[] {
            "Test",
            "Outpost Center Rush Drive Only"
        };

        public static final Rotation2d rotation45  = Rotation2d.fromDegrees(45);
        public static final Rotation2d rotation135 = Rotation2d.fromDegrees(135);
        public static final Rotation2d rotation225 = Rotation2d.fromDegrees(225);
        public static final Rotation2d rotation315 = Rotation2d.fromDegrees(315);

        public static final List<Rotation2d> BUMP_ROTATIONS = new ArrayList<Rotation2d>() {{
            Rotation2d rotation1 = rotation45;
            add(rotation1);
            Rotation2d rotation2 = rotation135;
            add(rotation2);
            Rotation2d rotation3 = rotation225;
            add(rotation3);
            Rotation2d rotation4 = rotation315;
            add(rotation4);
        }};

        public static final double TOWER_ALIGNMENT_OFFSET = 1;
        public static final double TOWER_ALIGNMENT_OFFSET_Y = 0;
    }

    public static final class MAXSwerveModuleConstants {
        // https://www.revrobotics.com/rev-21-3005/
        private enum SwerveGearing {
            LOW         (12, 22, 4.12, 4.92),
            MEDIUM      (13, 22, 4.46, 5.33),
            HIGH        (14, 22, 4.8, 5.5/*5.74*/),

            EXTRA_HIGH_1(14, 21, 5.03, 6.01),
            EXTRA_HIGH_2(14, 20, 5.28, 6.32),
            EXTRA_HIGH_3(15, 20, 5.66, 6.77),
            EXTRA_HIGH_4(16, 20, 6.04, 7.22),
            EXTRA_HIGH_5(16, 19, 6.36, 7.60);

            private final double 
                pinionTeeth, 
                spurTeeth, 
                maxSpeedNeo,
                maxSpeedVortex;

            SwerveGearing(
                int pinionTeeth, 
                int spurTeeth, 
                double maxSpeedNeo,
                double maxSpeedVortex)
            {
                this.pinionTeeth = pinionTeeth;
                this.spurTeeth = spurTeeth;
                this.maxSpeedNeo = maxSpeedNeo;
                this.maxSpeedVortex = maxSpeedVortex;
            }
            
        }

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.HIGH;

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        // **********************************************************************MAX SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.4642497827983136*2.0);
        // **********************************************************************MAX SWERVE**********************
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * CURRENT_GEARING.spurTeeth) / (CURRENT_GEARING.pinionTeeth * 15.0);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.256;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0.255;
        public static final double DRIVING_FF = 0.20217;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;
        public static final GainConstants DRIVING_PID = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_FF,
            DRIVING_MIN_OUTPUT,
            DRIVING_MAX_OUTPUT
        );

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1.5;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 1;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;
        public static final GainConstants TURNING_PID = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_FF,
            TURNING_MIN_OUTPUT,
            TURNING_MAX_OUTPUT
        );

        public static final int NEO_CURRENT_LIMIT = 50; // amps
        public static final int VORTEX_CURRENT_LIMIT = 80; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

    }

    public static final class MK5nSwerveModuleConstants {
        // Driving motors CAN IDs
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;
        public static final int DRIVE_MOTOR_TYPE = 60;

        // Turning motors CAN IDs
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;
        public static final int TURN_MOTOR_TYPE = 44;

        // CANcoders CAN IDs
        public static final int FRONT_LEFT_CANCODER_CAN_ID = 10;
        public static final int REAR_LEFT_CANCODER_CAN_ID = 11;
        public static final int FRONT_RIGHT_CANCODER_CAN_ID = 9;
        public static final int REAR_RIGHT_CANCODER_CAN_ID = 12;

        private enum SwerveGearing {

            R1(7.03),
            R2(6.03),
            R3(5.27);

            private final double gearRatio;

            SwerveGearing(double gearRatio) {
                this.gearRatio = gearRatio;
            }
            
        };

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.R2;

        public static final double FRONT_LEFT_TURN_ENCODER_OFFSET = -0.206787;
        public static final double FRONT_RIGHT_TURN_ENCODER_OFFSET = 0.367432;
        public static final double REAR_LEFT_TURN_ENCODER_OFFSET = 0.275391;
        public static final double REAR_RIGHT_TURN_ENCODER_OFFSET = 0.331299;


        // inversions for different motors may be needed for dual type
        public static final boolean INVERT_FRONT_LEFT_DRIVE = true;
        public static final boolean INVERT_FRONT_RIGHT_DRIVE = false;
        public static final boolean INVERT_BACK_LEFT_DRIVE = false;
        public static final boolean INVERT_BACK_RIGHT_DRIVE = true;

        public static final double TURNING_MOTOR_REDUCTION = 26.09;

        public static final double MAX_TURNING_MOTOR_VELOCITY_RADIANS_PER_SEC = 400;

        public static final boolean INVERT_TURNING_MOTOR = false;

        public static final boolean INVERT_TURNING_CANCODER = false;

        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = KrakenMotorConstants.KRAKENX60_FREE_SPEED_RPM_FOC / 60;

        // **********************************************************************MK5n SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(2 * 2); //TODO: run wheel radius characterization
        // **********************************************************************MK5n SWERVE**********************
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double DRIVE_GEAR_RATIO = CURRENT_GEARING.gearRatio;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = WHEEL_CIRCUMFERENCE_METERS; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI); // radians per second

        public static final double DRIVING_MOTOR_STATOR_LIMIT_AMPS = 80.0;
        public static final double DRIVING_MOTOR_SUPPLY_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_STATOR_LIMIT_AMPS = 40.0;
        public static final double TURNING_MOTOR_SUPPLY_LIMIT_AMPS = 40.0;
        public static final double DRIVING_MOTOR_TORQUE_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_TORQUE_LIMIT_AMPS = 40.0;

        public static final double DRIVING_P = 20.0; //20.0
        public static final double DRIVING_I = 6.0; //6
        public static final double DRIVING_D = 1.5; //1.5
        public static final double DRIVING_S = 0.0;
        public static final double DRIVING_V = 0.124; //0.124

        public static final GainConstants DRIVING_GAINS = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_S,
            DRIVING_V,
            0.0
        );

        public static final LoggedGainConstants LOGGED_DRIVING_GAINS = new LoggedGainConstants(MK5nSwerveModuleConstants.DRIVING_GAINS, "Swerve/Drive");

        public static final double TURNING_P = 700.0; //225
        public static final double TURNING_I = 0.0; //13
        public static final double TURNING_D = 12.0; //8
        public static final double TURNING_S = 0.0;

        public static final GainConstants TURNING_GAINS = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_S,
            0.0,
            0.0
        );

        public static final LoggedGainConstants LOGGED_TURNING_GAINS = new LoggedGainConstants(MK5nSwerveModuleConstants.TURNING_GAINS, "Swerve/Turn");

        public static final double DRIFT_MULTIPLIER = 4.0;

        public static final LoggedTunableConstant LOGGED_DRIFT_MULTIPLIER = new LoggedTunableConstant("Swerve/DriftMultiplier", DRIFT_MULTIPLIER);
    }

    public static final class MK4cSwerveModuleConstants {
        // Driving motors CAN IDs
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 4;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 7;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 10;

        // Turning motors CAN IDs
        public static final int FRONT_LEFT_TURNING_CAN_ID = 5;
        public static final int REAR_LEFT_TURNING_CAN_ID = 8;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 11;

        // CANcoders CAN IDs
        public static final int FRONT_LEFT_CANCODER_CAN_ID = 6;
        public static final int REAR_LEFT_CANCODER_CAN_ID = 9;
        public static final int FRONT_RIGHT_CANCODER_CAN_ID = 3;
        public static final int REAR_RIGHT_CANCODER_CAN_ID = 12;

        private enum SwerveGearing {

            L1(7.13),
            L2(5.9),
            L3(5.36);

            private final double gearRatio;

            SwerveGearing(double gearRatio) {
                this.gearRatio = gearRatio;
            }
            
        };

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.L2;

        public static final double FRONT_LEFT_TURN_ENCODER_OFFSET = -0.287354;
        public static final double FRONT_RIGHT_TURN_ENCODER_OFFSET = -0.116732;
        public static final double REAR_LEFT_TURN_ENCODER_OFFSET = 0.282227;
        public static final double REAR_RIGHT_TURN_ENCODER_OFFSET = 0.080322;

        public static final double TURNING_MOTOR_REDUCTION = 12.8;

        public static final boolean INVERT_TURNING_MOTOR = false;

        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = KrakenMotorConstants.KRAKENX60_FREE_SPEED_RPM_FOC / 60;
        // **********************************************************************MK4c SWERVE**********************
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.9548910849480585 * 2);
        // **********************************************************************MK4c SWERVE**********************
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS)
                / CURRENT_GEARING.gearRatio; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / CURRENT_GEARING.gearRatio); // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI); // radians per second

        public static final double DRIVING_MOTOR_STATOR_LIMIT_AMPS = 80.0;
        public static final double DRIVING_MOTOR_SUPPLY_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_STATOR_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_SUPPLY_LIMIT_AMPS = 80.0;
        public static final double DRIVING_MOTOR_TORQUE_LIMIT_AMPS = 80.0;
        public static final double TURNING_MOTOR_TORQUE_LIMIT_AMPS = 40.0;

        public static final double DRIVING_P = 0.02;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_S = 0.07193;
        public static final double DRIVING_V = 0.11441;

        public static final GainConstants DRIVING_GAINS = new GainConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_S,
            DRIVING_V,
            0.0
        );

        public static final double TURNING_P = 23.0;
        public static final double TURNING_I = 0.0;
        public static final double TURNING_D = 0.2;
        public static final double TURNING_S = 0.30000900000007824;

        public static final GainConstants TURNING_GAINS = new GainConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_S,
            0.0,
            0.0
        );

    }
    
    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;

        public static final double DRIVER_ALIGN_CANCEL_DEADBAND = 0.15;

        public enum DriverMode {
            SINGLE,
            DEV,
            CALIBRATION
        }

        public static final DriverMode DRIVER_MODE = DriverMode.DEV;

    }

    public static final class NeoMotorConstants {
        public static final double VORTEX_FREE_SPEED_RPM = 6784;

        public static ArrayList<SparkBase> motors = new ArrayList<SparkBase>();

        public static final boolean SAFE_SPARK_MODE = false;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 32766;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 15;
        // This gets filled out as motors are created on the robot
        public static final HashMap<Integer, Neo> NEO_MOTOR_MAP = new HashMap<Integer, Neo>();

        public static final HashMap<Integer, String> CAN_ID_MAP = new HashMap<Integer, String>() {{
            /*  1  */ put(MAXSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontRightDrive");
            /*  2  */ put(MAXSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID, "FrontRightTurn");
            /*  3  */ put(MAXSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontLeftDrive");
            /*  4  */ put(MAXSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID, "FrontLeftTurn");
            /*  5  */ put(MAXSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID, "RearLeftDrive");
            /*  6  */ put(MAXSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID, "RearLeftTurn");
            /*  7  */ put(MAXSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID, "RearRightDrive");
            /*  8  */ put(MAXSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID, "RearRightTurn");
        }};

        public static final HashMap<String, List<Neo>> NEO_MOTOR_GROUPS = new HashMap<String, List<Neo>>();

        public static Map<String, List<Neo>> initializeMotorGroupMap() {
            // NEO_MOTOR_GROUPS.put("Drive", new ArrayList<Neo>() {{
            //     add(NEO_MOTOR_MAP.get(DriveConstants.FRONT_LEFT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.REAR_LEFT_DRIVING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.REAR_RIGHT_DRIVING_CAN_ID));
            // }});
            // NEO_MOTOR_GROUPS.put("Turn", new ArrayList<Neo>() {{
            //     add(NEO_MOTOR_MAP.get(DriveConstants.FRONT_LEFT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.REAR_LEFT_TURNING_CAN_ID));
            //     add(NEO_MOTOR_MAP.get(DriveConstants.REAR_RIGHT_TURNING_CAN_ID));
            // }});

            return NEO_MOTOR_GROUPS;
        }

    }

    public static final class KrakenMotorConstants {

        public static final double KRAKENX60_FREE_SPEED_RPM = 6000;
        public static final double KRAKENX60_FREE_SPEED_RPM_FOC = 5800;

        public static final double KRAKENX44_FREE_SPEED_RPM = 7530;

        public static final double TALONFX_FAST_UPDATE_FREQ_HZ = 75;// TODO: FIND THE SWEET SPOT
        public static final double TALONFX_MID_UPDATE_FREQ_HZ = 50; // TODO: FIND THE SWEET SPOT
        public static final double TALONFX_SLOW_UPDATE_FREQ_HZ = 4; // TODO: FIND THE SWEET SPOT

        public static final HashMap<Integer, Kraken> KRAKEN_MOTOR_MAP = new HashMap<Integer, Kraken>();

        public static final HashMap<String, List<Kraken>> KRAKEN_MOTOR_GROUPS = new HashMap<String, List<Kraken>>();

        public static Map<String, List<Kraken>> initializeMotorGroupMap() {
            KRAKEN_MOTOR_GROUPS.put("Drive", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.FRONT_LEFT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.REAR_LEFT_DRIVING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.REAR_RIGHT_DRIVING_CAN_ID));
            }});
            KRAKEN_MOTOR_GROUPS.put("Turn", new ArrayList<Kraken>() {{
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.FRONT_LEFT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.FRONT_RIGHT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.REAR_LEFT_TURNING_CAN_ID));
                add(KRAKEN_MOTOR_MAP.get(MK5nSwerveModuleConstants.REAR_RIGHT_TURNING_CAN_ID));
            }});

            return KRAKEN_MOTOR_GROUPS;
        }

    }

    public static final class CANConstants {
        public static final CANBus RIO_BUS = CANBus.roboRIO();
        public static final CANBus DRIVEBASE_BUS =  new CANBus("Drivebase");
    }

    public static final class CANCoderConstants {

        public static final double ENCODER_UPDATE_FREQ_HZ = 100; // TODO: FIND THE SWEET SPOT

    }

    public static final class PigeonConstants {

        public static final double PIGEON_FAST_UPDATE_FREQ_HZ = 250; // TODO: FIND THE SWEET SPOT
        public static final double YAW_OFFSET = 180.0;
        public static final double PITCH_OFFSET = 0.0;
        public static final double ROLL_OFFSET = 0.0;
        public static final MountPoseConfigs POSE_CONFIGS = new MountPoseConfigs(){{
            withMountPoseYaw(YAW_OFFSET);
            withMountPosePitch(PITCH_OFFSET);
            withMountPoseRoll(ROLL_OFFSET);
        }};
    }

    public static final class CANRangeConstants {
        public static final double UPDATE_FREQUENCY = 0.03;

        public static final int CANRANGE_ID = 28;
    }
    
    public static final class CameraConstants {

        public static final int ENABLED_THROTTLE = 0; // Process every frame
        public static final int DISABLED_THROTTLE = 50; // Process 1 of every n frames

        public static final Pose3d LL4_LEFT_POSE = new Pose3d(
            -0.282,
            0.308,
            0.203,
            new Rotation3d(
                Units.degreesToRadians(180d),
                Units.degreesToRadians(25d),
                Units.degreesToRadians(0)
            )
        );

        public static final Pose3d LL3G_POSE = new Pose3d(
            -0.292,
            0.021,
            0.339,
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(25d),
                Units.degreesToRadians(90d)
            )
        );

        public static final Pose3d LLHAILO_POSE = new Pose3d(
            0.281,
            0.290,
            0.200,
            new Rotation3d(
                Units.degreesToRadians(180d),
                Units.degreesToRadians(25d),
                Units.degreesToRadians(-34.5)
            )
        );

        public static final Pose3d LL4_RIGHT_POSE = new Pose3d(
            0.076,
            0.305,
            0.197,
            new Rotation3d(
                Units.degreesToRadians(180d),
                Units.degreesToRadians(25d),
                Units.degreesToRadians(-105)
            )
        );


        // UPDATE LATER once we know how many cameras we have and their positions
     //   public static Pose3d LL3G_POSE = new Pose3d(
       //     -0.312655,
         //   -0.120619,
     //       0.197874,
       //     new Rotation3d(
     //           Units.degreesToRadians(0), // -2.303
       //         Units.degreesToRadians(15),
         //       Units.degreesToRadians(-141.77)
      //      )
       /// );
       
        // UPDATE LATER once we know how many cameras we have and their positions
      //  public static Pose3d LL4_POSE = new Pose3d(
        //    -0.312655,
          //  0.120619,
       //     0.197874,
         //   new Rotation3d(
           //     Units.degreesToRadians(0), // 2.303
             //   Units.degreesToRadians(15),
               // Units.degreesToRadians(141.77)
         //   )
       // );
    }

    public static final class ClimbConstants {
        
        public static final int CAN_ID = 13;
        public static final int MOTOR_TYPE = 60;
        
        public static final boolean MOTOR_INVERTED = false;
        
        public static final double GEAR_RATIO = 9.0;
        public static final double MAX_DISPLACEMENT_METERS = 0.58;
        public static final double STOW_POSITION_METERS = 0.0;
        public static final double CLIMB_POSITION_METERS = 0.0;
        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;
        
        public static final double CURRENT_LIMIT = 120.0;
        public static final boolean BRAKE_MOTOR = true;
        
        //TODO: tune these gains
        public static final double P_FAST = FieldConstants.IS_REAL ? 0d : 0d; // used to pull robot up
        public static final double I_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double A_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double S_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double V_FAST = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double G_FAST = FieldConstants.IS_REAL ? 0d : 0d;

        public static final double P_SLOW = FieldConstants.IS_REAL ? 0d : 0d; // used to bring hooks up
        public static final double I_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double D_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double A_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double S_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double V_SLOW = FieldConstants.IS_REAL ? 0d : 0d;
        public static final double G_SLOW = FieldConstants.IS_REAL ? 0d : 0d; //TODO: calculate g gain for weight of the robot

        public static final GainConstants SLOW_GAINS = new GainConstants(
            P_SLOW,
            I_SLOW,
            D_SLOW,
            A_SLOW,
            S_SLOW,
            V_SLOW,
            G_SLOW
        );

        public static final GainConstants FAST_GAINS = new GainConstants(
            P_SLOW,
            I_SLOW,
            D_SLOW,
            A_SLOW,
            S_SLOW,
            V_SLOW,
            G_SLOW
        );
        
        public static final double VELOCITY = Math.PI;
        public static final double ACCELERATION = 20d;
        public static final double JERK = 200d;
        
        public static final double VELOCITY_DEADBAND_MPS = 0.2;
        public static final double DEADBAND_METERS = 0.2;
    }

    public static final class TurretConstants {
        public static final int MOTOR_CAN_ID = 17;
        public static final int MOTOR_TYPE = 44;

        public static final int ENCODER_CAN_ID = 18;

        public static final boolean BRAKE_MOTOR = true;
        public static final boolean MOTOR_INVERTED = true; 
        public static final boolean CANCODER_INVERTED = true;
        public static final double CURRENT_LIMIT = 50;

        public static final double ABSOLUTE_ENCODER_OFFSET = -0.686768;
        public static final double MOTOR_GEAR_RATIO = 50.0;
        public static final double MOTOR_TO_ENCODER_GEAR_RATIO = 5.0;
        public static final double ENCODER_TO_MECH_GEAR_RATIO = 10.0;
        public static final double DISCONTINUITY_POINT = 1.0;
        public static final double ROBOT_TURRET_OFFSET = Units.degreesToRadians(4.5);
        public static final double ANGULAR_OFFSET = Units.degreesToRadians(0.0);

        //conversions from rotations to radians
        public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI; 
        public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI;
        public static final double ENCODER_POSITION_CONVERSION_FACTOR = 2 * Math.PI;

        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(354); 

        public static final Rotation2d HUB_POSITION = new Rotation2d(Units.degreesToRadians(0.0)); // TODO: find this value

        public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);
        public static final double PASS_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(5);

        // TODO: tune these values
        public static final double P = FieldConstants.IS_REAL ? 300d : 0; 
        public static final double I = FieldConstants.IS_REAL ? 0 : 0;
        public static final double D = FieldConstants.IS_REAL ? 25d : 0;
        public static final double A = FieldConstants.IS_REAL ? 0 : 0;
        public static final double S = FieldConstants.IS_REAL ? 0 : 0;
        public static final double V = FieldConstants.IS_REAL ? 0 : 0; 
        public static final double G = FieldConstants.IS_REAL ? 0 : 0;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        public static final LoggedGainConstants LOGGED_TURRET_GAINS = new LoggedGainConstants(TurretConstants.GAINS, "Shooter/Turret/Gains");
        public static final LoggedTunableConstant LOGGED_TEST_POSITION_DEGREES = new LoggedTunableConstant("Shooter/Turret/PositionDegrees", 0.0);

        public static final double VELOCITY = 1.0;
        public static final double ACCELERATION = 1.0;
        public static final double JERK = 0.5;

        public static final Transform3d TURRET_TO_ROBOT_TRANSFORM3D = new Transform3d(0.000, 0.000, 0.000, Rotation3d.kZero);
        public static final Transform2d TURRET_TO_ROBOT_TRANSFORM2D = new Transform2d(
            TURRET_TO_ROBOT_TRANSFORM3D.getX(),
            TURRET_TO_ROBOT_TRANSFORM3D.getY(),
            TURRET_TO_ROBOT_TRANSFORM3D.getRotation().toRotation2d()
        );
    }

    public static final class FlywheelConstants {
        public static final int CAN_ID = 20;
        public static final int MOTOR_TYPE = 60;

        public static final boolean BRAKE_MOTOR = false;
        public static final boolean MOTOR_INVERTED = true;
        public static final double CURRENT_LIMIT = 50;

        public static final double GEAR_RATIO = 1.0;

        public static final double VELOCITY_DEADBAND_RPM = 50; 
        public static final double PASS_VELOCITY_DEADBAND_RPM = 120;

        public static final double EJECT_RPM = 30.0;
        public static final double HUB_RPM = 0.0; // TODO: find this value

        public static final double CIRCUMFERENCE_METERS = Units.inchesToMeters(4 * Math.PI);

        // TODO: tune these values
        // if it's not in sim use first values else use second values: note to self delete this afterwards
        public static final double P = FieldConstants.IS_REAL ? 0.5 : 0;
        public static final double I = FieldConstants.IS_REAL ? 0 : 0;
        public static final double D = FieldConstants.IS_REAL ? 0 : 0;
        public static final double A = FieldConstants.IS_REAL ? 0 : 0;
        public static final double S = FieldConstants.IS_REAL ? 0.169921875 : 0;
        public static final double V = FieldConstants.IS_REAL ? 0.11959999799728394 : 0;
        public static final double G = FieldConstants.IS_REAL ? 0 : 0;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        public static final LoggedGainConstants LOGGED_FLYWHEEL_GAINS = new LoggedGainConstants(FlywheelConstants.GAINS, "Shooter/Flywheel/Gains");
        public static final LoggedTunableConstant LOGGED_SPEED_RPM = new LoggedTunableConstant("Shooter/Flyweel/SpeedRPM", 0.0);
    }

    public static final class HoodConstants {
        public static final int CAN_ID = 19;
        public static final int MOTOR_TYPE = 44;

        public static final boolean BRAKE_MOTOR = true;
        public static final boolean MOTOR_INVERTED = true;
        public static final double CURRENT_LIMIT = 150;

        public static final double GEAR_RATIO = 32.45;

        public static final double MIN_ANGLE_RADIANS = 0.0;
        public static final double MAX_ANGLE_RADIANS = 0.45099; 
        
        public static final double STOW_POSITION_RADIANS = 0.0;
        public static final double EJECT_POSITION_RADIANS = 0.5;
        public static final double HUB_POSITION_RADIANS = 0.0; // TODO: find this value

        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;

        public static final double POSITION_DEADBAND_RADIANS = 0.1;
        public static final double PASS_POSITION_DEADBAND_RADIANS = 0.5;
        
        // TODO: tune these values
        public static final double P = FieldConstants.IS_REAL ? 1700 : 0;
        public static final double I = FieldConstants.IS_REAL ? 350: 0;
        public static final double D = FieldConstants.IS_REAL ? 45 : 0;
        public static final double A = FieldConstants.IS_REAL ? 0 : 0;
        public static final double S = FieldConstants.IS_REAL ? 0 : 0;
        public static final double V = FieldConstants.IS_REAL ? 0 : 0;
        public static final double G = 0.0; //21.5 amps

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        public static final LoggedGainConstants LOGGED_HOOD_GAINS = new LoggedGainConstants(HoodConstants.GAINS, "Shooter/Hood/Gains");
        public static final LoggedTunableConstant LOGGED_TEST_POSITION_DEGREES = new LoggedTunableConstant("Shooter/Hood/PositionDegrees", 0.0);

        public static final double VELOCITY = 1.6;
        public static final double ACCELERATION = 3d;
        public static final double JERK = 40d;
    }

    public static final class IntakeConstants {
        public static final int CAN_ID = 13;
        public static final int MOTOR_TYPE = 60;
        public static final double GEAR_RATIO = 3.33333;

        public static final boolean BRAKE_MOTOR = false;
        public static final boolean MOTOR_INVERTED = true;
        public static final double CURRENT_LIMIT = 60;

        public static final double INTAKE_PERCENT = 0.5;

        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;

        public static final LoggedTunableConstant TEST_PERCENT = new LoggedTunableConstant("Intake/Percent", INTAKE_PERCENT);
    }

    public static final class ExtenderConstants {
        public static final int CAN_ID = 14;
        public static final int MOTOR_TYPE = 44;
        public static final double GEAR_RATIO = 60.0;

        public static final boolean BRAKE_MOTOR = true;
        public static final boolean MOTOR_INVERTED = false;
        public static final double CURRENT_LIMIT = 60.0;

        public static final double MIN_ANGLE_RADIANS = 0.0;
        public static final double MAX_ANGLE_RADIANS = 2.19;

        public static final double STOW_POSITION_RADIANS = 0.2;
        public static final double INTAKE_POSITION_RADIANS = MAX_ANGLE_RADIANS;
        public static final double HALF_STOW_RADIANS = INTAKE_POSITION_RADIANS / 2;
        public static final double WIGGLE_RADIANS = MAX_ANGLE_RADIANS - 1.5;

        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;

        public static final double DEADBAND_RADIANS = 0.1;

        public static final double P = FieldConstants.IS_REAL ? 100 : 0;
        public static final double I = FieldConstants.IS_REAL ? 15 : 0;
        public static final double D = FieldConstants.IS_REAL ? 10 : 0;
        public static final double A = FieldConstants.IS_REAL ? 0 : 0;
        public static final double S = FieldConstants.IS_REAL ? 0 : 0;
        public static final double V = FieldConstants.IS_REAL ? 0 : 0;
        public static final double G = FieldConstants.IS_REAL ? 0 : 0;

        public static final GainConstants GAINS = new GainConstants(
            P,
            I,
            D,
            A,
            S,
            V,
            G
        );

        public static final LoggedGainConstants LOGGED_GAINS = new LoggedGainConstants(ExtenderConstants.GAINS, "Extender/ExtenderGains");

        public static final LoggedTunableConstant LOGGED_TEST_POSITION = new LoggedTunableConstant("Extender/ExtenderPositionDegrees", MIN_ANGLE_RADIANS);

        public static final LoggedTunableBoolean LOGGED_BRAKE_MODE = new LoggedTunableBoolean("Extender/BrakeExtender", ExtenderConstants.BRAKE_MOTOR);
    }

    public static final class KickerConstants {
        public static final int CAN_ID = 16;
        public static final int MOTOR_TYPE = 60;

        public static final int CANRANGE_ID = 28;
        
        public static final boolean MOTOR_INVERTED = false;
        
        public static final double GEAR_RATIO = (22 / 12);
        public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;
        
        public static final double CURRENT_LIMIT = 40.0;
        public static final boolean BRAKE_MOTOR = false;

        public static final double DEBOUNCER_TIME = 0.5;
        
        public static final double DEADBAND_RADIANS = 0.0;

        public static final double IDLE_PERCENT = 0.0;
        public static final double REVERSE_PERCENT = -0.3;
        public static final double FORWARD_PERCENT = 0.3;
        
        public static final LoggedTunableConstant LOGGED_KICKER_PERCENT = new LoggedTunableConstant("Hopper/Kicker/PercentOutput", 0.0);
    }

    public static final class GeneralHardwareConstants {
        public static final boolean SAFE_HARDWARE_MODE = false;
        public static final double TIMEOUT_SECONDS = 1.0;
    }

    public static final class SpindexerConstants {
        public static final int CAN_ID = 15;

        public static final boolean BRAKE_MOTOR = false;
        public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI;

        public static final double CURRENT_LIMIT = 50.0;

        public static final boolean MOTOR_INVERTED = true;
        public static final int MOTOR_TYPE = 44;

        public static final double IDLE_PERCENT = 0.0;
        public static final double CLOCKWISE_PERCENT = -0.6;
        public static final double COUNTERCLOCKWISE_PERCENT = 0.6;
        public static final double FEED_PERCENT = 0.6;
        
        public static final double GEAR_RATIO = 3.0;

        public static final LoggedTunableConstant TEST_PERCENT = new LoggedTunableConstant("Hopper/Spindexer/PercentOutput", SpindexerConstants.FEED_PERCENT);
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();
        public static boolean IS_REAL = !IS_SIMULATION;

        public static Rotation3d zeroRotation = new Rotation3d();

        public static final double FIELD_LENGTH = 16.5890;
        public static final double FIELD_WIDTH = 8.0756;

        public static final Pose2d ALLIANCE_LINE_SHOT_RED_LEFT = new Pose2d(4.343, 7.354, new Rotation2d(0.0));
        public static final Pose2d ALLIANCE_LINE_SHOT_RED_RIGHT = new Pose2d(4.312, 0.510, new Rotation2d(0.0));

        public static final double CENTER_LINE_X = 8.2499;
        public static final double ALLIANCE_LINE_BLUE = 4.0271;
        public static final double ALLIANCE_LINE_RED = 12.5488;

        public static final double CURRENT_ALLIANCE_LINE() {
            double line = Robot.isRedAlliance()? ALLIANCE_LINE_RED : ALLIANCE_LINE_BLUE;
            return line;
        }

        public static final double OPPOSING_ALLIANCE_LINE() {
            double line = Robot.isRedAlliance()? ALLIANCE_LINE_BLUE : ALLIANCE_LINE_RED;
            return line;
        }

        public static final Translation3d HUB_RED = new Translation3d(4.6209, 4.041, 1.8288);
        
        public static final List<Translation3d> HUBS = new ArrayList<Translation3d>() {{
            add(HUB_RED);
            add(PoseCalculations.mirrorTranslation3d(HUB_RED));
        }};

        public static final Translation3d GET_HUB() {
            int index = Robot.isRedAlliance() ? 0 : 1;
            return HUBS.get(index);
        }
        
        public static final Translation3d PASS_LEFT_LOCATION = new Translation3d(4.0566, 5.5642, 0);
        public static final Translation3d PASS_RIGHT_LOCATION = PoseCalculations.flipTranslation3dY(PASS_LEFT_LOCATION);

        public static final List<Translation3d> PASS_LEFT_LOCATIONS = new ArrayList<Translation3d>() {{
            add(PASS_LEFT_LOCATION);
            add(PoseCalculations.mirrorTranslation3d(PASS_LEFT_LOCATION));
        }};

        public static final List<Translation3d> PASS_RIGHT_LOCATIONS = new ArrayList<Translation3d>() {{
            add(PASS_RIGHT_LOCATION);
            add(PoseCalculations.mirrorTranslation3d(PASS_RIGHT_LOCATION));
        }};

        public static final Translation3d GET_PASS_LEFT() {
            int index = Robot.isRedAlliance() ? 0 : 1;
            return PASS_LEFT_LOCATIONS.get(index);
        }

        public static final Translation3d GET_PASS_RIGHT() {
            int index = Robot.isRedAlliance() ? 0 : 1;
            return PASS_RIGHT_LOCATIONS.get(index);
        }

        public static final Pose2d OUTPOST_RED = new Pose2d(0.6378, 0.0, Rotation2d.fromDegrees(0.0));
        public static final List<Pose2d> OUTPOSTS = new ArrayList<Pose2d>() {{
            Pose2d OutpostRed = OUTPOST_RED;
            //RED Outpost
            add(OutpostRed);
            //BLUE Outpost
            add(PoseCalculations.mirrorPose(OutpostRed));
        }};

        public static final List<Pose2d> GET_OUTPOST() {
            int startindex = Robot.isRedAlliance()? 0 : 1;
            return OUTPOSTS.subList(startindex, startindex + 1);
        }
        
        public static final Pose2d DEPOT_CENTER_RED = new Pose2d(0.6958, 5.9371, Rotation2d.fromDegrees(0.0));
        public static final List<Pose2d> DEPOTS = new ArrayList<Pose2d>() {{
            Pose2d DepotRed = DEPOT_CENTER_RED;
            //RED Depot
            add(DepotRed);
            //BLUE Depot
            add(PoseCalculations.mirrorPose(DepotRed));
        }};

        public static final List<Pose2d> GET_DEPOT() {
            int startindex = Robot.isRedAlliance()? 0 : 1;
            return DEPOTS.subList(startindex, startindex + 1);
        }

        public static final Pose2d TOWER_CENTER_RED = new Pose2d(1.1082, 3.8304, Rotation2d.fromDegrees(0.0));
        public static final double TOWER_WIDTH = 1.1938;
        public static final double TOWER_POST_DISTANCE = 0.8573;
        public static final double TOWER_RUNG_3 = 1.6002;
        public static final double TOWER_RUNG_2 = 1.1430;
        public static final double TOWER_RUNG_1 = 0.6858;
        public static final List<Pose2d> TOWERS = new ArrayList<Pose2d>() {{
            //RED Tower
            Pose2d TowerRed = TOWER_CENTER_RED;
            add(TowerRed);
            //Posts
            add(PoseCalculations.getTowerDimensions(TowerRed, "LEFT-POST"));
            add(PoseCalculations.getTowerDimensions(TowerRed, "RIGHT-POST"));
            //Sides
            add(PoseCalculations.getTowerDimensions(TowerRed, "LEFT-SIDE"));
            add(PoseCalculations.getTowerDimensions(TowerRed, "RIGHT-SIDE"));
            //BLUE Tower
            Pose2d TowerBlue = PoseCalculations.mirrorPose(TOWER_CENTER_RED);
            add(TowerBlue);
            //Posts
            add(PoseCalculations.getTowerDimensions(TowerBlue, "LEFT-POST"));
            add(PoseCalculations.getTowerDimensions(TowerBlue, "RIGHT-POST"));
            //Sides
            add(PoseCalculations.getTowerDimensions(TowerBlue, "LEFT-SIDE"));
            add(PoseCalculations.getTowerDimensions(TowerBlue, "RIGHT-SIDE"));
        }};


        public static final Pose2d GET_TOWER_CENTER() {
            int index = Robot.isRedAlliance()? 0 : 5;
            return TOWERS.get(index);
        }

        public static final List<Pose2d> GET_TOWER_POSTS() {
            int startindex = Robot.isRedAlliance()? 1 : 6;
            return TOWERS.subList(startindex, startindex + 1);
        }

        public static final List<Pose2d> GET_TOWER_SIDES() {
            int startindex = Robot.isRedAlliance()? 3 : 8;
            return TOWERS.subList(startindex, startindex + 1);
        }

        public static final Pose3d TRENCH_CENTER_SIDE1_RED = new Pose3d(4.6145, 0.5566, 0.5651, zeroRotation);
        public static final double TRENCH_LENGTH = 1.1859;
        public static final double TRENCH_WIDTH = 1.5772;
        public static final List<Pose2d> TRENCHES = new ArrayList<Pose2d>() {{
            //RED TRENCHES
            //Side 1
            Pose3d trenchRed1 = TRENCH_CENTER_SIDE1_RED;
            add(trenchRed1.toPose2d());
            add(PoseCalculations.getTrenchDimension(trenchRed1, 'L'));
            add(PoseCalculations.getTrenchDimension(trenchRed1, 'R'));
            add(PoseCalculations.getTrenchDimension(trenchRed1, 'U'));
            add(PoseCalculations.getTrenchDimension(trenchRed1, 'D'));
            //Side 2
            Pose3d trenchRed2 = PoseCalculations.flipPose3dY(TRENCH_CENTER_SIDE1_RED);
            add(trenchRed2.toPose2d());
            add(PoseCalculations.getTrenchDimension(trenchRed2, 'L'));
            add(PoseCalculations.getTrenchDimension(trenchRed2, 'R'));
            add(PoseCalculations.getTrenchDimension(trenchRed2, 'U'));
            add(PoseCalculations.getTrenchDimension(trenchRed2, 'D'));
            //Blue Trenches
            //Side 1
            Pose3d trenchBlue1 = PoseCalculations.mirrorPose3d(TRENCH_CENTER_SIDE1_RED);
            add(trenchBlue1.toPose2d());
            add(PoseCalculations.getTrenchDimension(trenchBlue1, 'L'));
            add(PoseCalculations.getTrenchDimension(trenchBlue1, 'R'));
            add(PoseCalculations.getTrenchDimension(trenchBlue1, 'U'));
            add(PoseCalculations.getTrenchDimension(trenchBlue1, 'D'));
            //Side 2
            Pose3d trenchBlue2 = PoseCalculations.flipPose3dY(trenchBlue1);
            add(trenchBlue2.toPose2d());
            add(PoseCalculations.getTrenchDimension(trenchBlue2, 'L'));
            add(PoseCalculations.getTrenchDimension(trenchBlue2, 'R'));
            add(PoseCalculations.getTrenchDimension(trenchBlue2, 'U'));
            add(PoseCalculations.getTrenchDimension(trenchBlue2, 'D'));
        }};

        public static final List<Pose2d> GET_ALLIANCE_TRENCHES() {
            int startindex = Robot.isRedAlliance()? 0 : 10;
            return TRENCHES.subList(startindex, startindex + 10);
        }

        public static final List<Pose2d> GET_OPPOSING_TRENCHES() {
            int startindex = Robot.isRedAlliance()? 8 : 0;
            return TRENCHES.subList(startindex, startindex + 8);
        }

        public static final Pose3d BUMP_CENTER_SIDE1_RED = new Pose3d(4.6104, 3.1185, 0.1654, zeroRotation);
        public static final double BUMP_LENGTH = 1.1276;
        public static final double BUMP_WIDTH = 1.8436;
        public static final List<Pose2d> BUMPS = new ArrayList<Pose2d>() {{
            //RED Bumps
            //Side 1
            Pose3d bumpRed1 = BUMP_CENTER_SIDE1_RED;
            add(bumpRed1.toPose2d());
            add(PoseCalculations.getBumpDimension(bumpRed1, 'L'));
            add(PoseCalculations.getBumpDimension(bumpRed1, 'R'));
            add(PoseCalculations.getBumpDimension(bumpRed1, 'U'));
            add(PoseCalculations.getBumpDimension(bumpRed1, 'D'));
            //Side 2
            Pose3d bumpRed2 = PoseCalculations.flipPose3dY(BUMP_CENTER_SIDE1_RED);
            add(bumpRed2.toPose2d());
            add(PoseCalculations.getBumpDimension(bumpRed2, 'L'));
            add(PoseCalculations.getBumpDimension(bumpRed2, 'R'));
            add(PoseCalculations.getBumpDimension(bumpRed2, 'U'));
            add(PoseCalculations.getBumpDimension(bumpRed2, 'D'));
            //BLUE Bumps
            //Side 1
            Pose3d bumpBlue1 = PoseCalculations.mirrorPose3d(BUMP_CENTER_SIDE1_RED);
            add(bumpBlue1.toPose2d());
            add(PoseCalculations.getBumpDimension(bumpBlue1, 'L'));
            add(PoseCalculations.getBumpDimension(bumpBlue1, 'R'));
            add(PoseCalculations.getBumpDimension(bumpBlue1, 'U'));
            add(PoseCalculations.getBumpDimension(bumpBlue1, 'D'));
            //Side 2
            Pose3d bumpBlue2 = PoseCalculations.flipPose3dY(bumpBlue1);
            add(bumpBlue2.toPose2d());
            add(PoseCalculations.getBumpDimension(bumpBlue2, 'L'));
            add(PoseCalculations.getBumpDimension(bumpBlue2, 'R'));
            add(PoseCalculations.getBumpDimension(bumpBlue2, 'U'));
            add(PoseCalculations.getBumpDimension(bumpBlue2, 'D'));
        }};

        public static final List<Pose2d> GET_ALLIANCE_BUMPS() {
            int startindex = Robot.isRedAlliance()? 0 : 8;
            return BUMPS.subList(startindex, startindex + 8);
        }

        public static final List<Pose2d> GET_OPPOSING_BUMPS() {
            int startindex = Robot.isRedAlliance()? 8 : 0;
            return BUMPS.subList(startindex, startindex + 8);
        }

        public static final List<Pose2d> ALLIANCE_LINE_LIST = new ArrayList<Pose2d>() {{
            add(ALLIANCE_LINE_SHOT_RED_LEFT);
            add(ALLIANCE_LINE_SHOT_RED_RIGHT);
            add(PoseCalculations.mirrorPose(ALLIANCE_LINE_SHOT_RED_LEFT));
            add(PoseCalculations.mirrorPose(ALLIANCE_LINE_SHOT_RED_RIGHT));
        }};
    }
}
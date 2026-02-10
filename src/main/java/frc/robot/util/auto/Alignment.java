package frc.robot.util.auto;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.PoseCalculations;

import java.util.function.Supplier;

public class Alignment {

    private final Swerve swerve;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentMode")
    private AlignmentMode alignmentMode = AlignmentMode.NONE;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentIndex")
    private int alignmentIndex = 0;

    private State xySetpoint = new State(0, 0);
    private State targetXY = new State(0, 0);

    private Pose2d profileStartingPose = Pose2d.kZero;
    private TrapezoidProfile xyProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(AutoConstants.HDC_XY_VELOCITY, AutoConstants.HDC_XY_ACCELERATION));
    // private LoggedTunableNumber translationVelocity = new LoggedTunableNumber("Alignment/TranslationVelocity", AutoConstants.HDC_XY_VELOCITY);
    // private LoggedTunableNumber translationAcceleration = new LoggedTunableNumber("Alignment/TranslationAcceleration", AutoConstants.HDC_XY_ACCELERATION);

    public Alignment(Swerve swerve) {
        this.swerve = swerve;

        // translationVelocity.onChanged().or(translationAcceleration.onChanged())
        //     .onTrue(Commands.runOnce(() -> {
        //         xyProfile = new TrapezoidProfile(new Constraints(translationVelocity.get(), translationAcceleration.get()));
        //     }).ignoringDisable(true));
    }

    public void resetHDC() {
        // Reset error and integral terms of HDC's PID controller
        AutoConstants.TELE_HDC.getThetaController().reset(swerve.getPose().getRotation().getRadians(), swerve.getRobotRelativeVelocity().omegaRadiansPerSecond);
        AutoConstants.TELE_HDC.getXController().reset();
        AutoConstants.TELE_HDC.getYController().reset();
    }

    public Command resetHDCCommand() {
        return Commands.runOnce(() -> resetHDC());
    }

    //edit when calc poses are done
    public enum AlignmentMode {
        NONE,
        TOWER,
        BUMP,
        TRENCH,
        STARTING_LINE
    }

    public void resetProfile() {
        xySetpoint.position = 0;
        targetXY.position = 0;
        profileStartingPose = Pose2d.kZero;
    }
  

    public Command resetProfileCommand() {
        return Commands.runOnce(() -> resetProfile());
    }

    public double normalizeTwoSpeeds(double controllerInput, double autoInput) {
        // Normalize HDC input between -1 and 1
        autoInput /= swerve.getMaxLinearVelocity();

        // Combine speeds and normalize them between -1 and 1
        return MathUtil.clamp(
            controllerInput + autoInput,
            -1,
            1
        );
    }

    public double normalizeSpeed(double autoInput) {
        // Normalize HDC input between -1 and 1
        autoInput /= swerve.getMaxLinearVelocity();

        // Combine speeds and normalize them between -1 and 1
        return MathUtil.clamp(
            autoInput,
            -1,
            1
        );
    }
    
    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds, ChassisSpeeds controllerSpeeds) {
        return new ChassisSpeeds(
            MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond)), 0.005),
            -MathUtil.applyDeadband((normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond)), 0.005),
            MathUtil.applyDeadband(autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity(), 0.005)
        );
    }

    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds autoSpeeds) {
        ChassisSpeeds normalizedSpeeds = new ChassisSpeeds(
            MathUtil.applyDeadband((normalizeSpeed(autoSpeeds.vyMetersPerSecond)), 0.01),
            -MathUtil.applyDeadband((normalizeSpeed(autoSpeeds.vxMetersPerSecond)), 0.01),
            MathUtil.applyDeadband(autoSpeeds.omegaRadiansPerSecond / swerve.getMaxAngularVelocity(), 0.005));
        // Logger.recordOutput("Subsystems/Swerve/NormalizedSpeeds", normalizedSpeeds);
        return normalizedSpeeds;
    }

    public Pose2d getProfiledPosition(Pose2d currentPose, Pose2d targetPose) {
        xySetpoint = xyProfile.calculate(0.02, xySetpoint, targetXY);
        double magnitude = xySetpoint.position;
        double theta = Math.atan2(targetPose.getY() - profileStartingPose.getY(), targetPose.getX() - profileStartingPose.getX());
        double xOffset = magnitude * Math.cos(theta);
        double yOffset = magnitude * Math.sin(theta);
        // use the calculated states and get the positions to create a pose2d
        return new Pose2d(profileStartingPose.getX() + xOffset, profileStartingPose.getY() + yOffset, targetPose.getRotation());
    }

    public ChassisSpeeds getProfiledAutoSpeeds(Pose2d position) {
        Pose2d currentPose = swerve.getPose();
        if (targetXY.position == 0 || !swerve.getDesiredPose().equals(position)) {
            // First run of alignment or change of target pose, set up profile
            xySetpoint.position = 0;
            double distanceToTarget = currentPose.getTranslation().getDistance(position.getTranslation());
            targetXY.position = distanceToTarget;
            ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeVelocity(), currentPose.getRotation());
            double vx = robotSpeeds.vxMetersPerSecond;
            double vy = robotSpeeds.vyMetersPerSecond;
            // Get magnitude of velocity vector
            double vMagnitude = Math.hypot(vx, vy);
            // Get direction of velocity vector with the origin at the robot
            double vTheta = Math.atan2(vy, vx);
            // Get direction of position vector with the origin at the robot
            double pTheta = Math.atan2(position.getY() - currentPose.getY(), position.getX() - currentPose.getX());
            // Get angle difference
            double theta = vTheta - pTheta;
            // Project velocity vector onto direction of position vector by calculating 
            // a new component of the velocity vector that acts in the direction of pTheta
            xySetpoint.velocity = MathUtil.clamp(vMagnitude * Math.cos(theta), 0, swerve.getMaxLinearVelocity());
            // Logger.recordOutput("Subsystems/Swerve/SetpointStartVelocity", xySetpoint.velocity);
            profileStartingPose = currentPose;
        }
        swerve.setDesiredPose(position);
        position = getProfiledPosition(currentPose, position);
        //Logger.recordOutput("Subsystems/Swerve/StepPosition", position);
        return AutoConstants.TELE_HDC.calculate(swerve.getPose(), position, 0, position.getRotation());
    }

    public ChassisSpeeds getControllerSpeeds(double controllerX, double controllerY) {
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                controllerY * (Robot.isRedAlliance() ? -1 : 1), 
                controllerX * (Robot.isRedAlliance() ? 1 : -1), 
                0, 
                swerve.getPose().getRotation()
            );
    }

    public ChassisSpeeds getTowerAlignmentSpeeds() {
        Pose2d towerCenter = PoseCalculations.behindTower(swerve.getPose())? new Pose2d(
            (FieldConstants.GET_TOWER_CENTER().getX() - AutoConstants.TOWER_ALIGNMENT_OFFSET), 
            FieldConstants.GET_TOWER_CENTER().getY(), 
            FieldConstants.GET_TOWER_CENTER().getRotation()
        ) : new Pose2d(
            (FieldConstants.GET_TOWER_CENTER().getX() + AutoConstants.TOWER_ALIGNMENT_OFFSET), 
            FieldConstants.GET_TOWER_CENTER().getY(), 
            FieldConstants.GET_TOWER_CENTER().getRotation()
        );
        Pose2d postLeft = PoseCalculations.behindTower(swerve.getPose())? new Pose2d(
            (FieldConstants.GET_TOWER_POSTS().get(2).getX() - AutoConstants.TOWER_ALIGNMENT_OFFSET),
            (FieldConstants.GET_TOWER_POSTS().get(2).getY() - AutoConstants.TOWER_ALIGNMENT_OFFSET_Y),
            FieldConstants.GET_TOWER_POSTS().get(2).getRotation()
        ) : new Pose2d(
            (FieldConstants.GET_TOWER_POSTS().get(2).getX() + AutoConstants.TOWER_ALIGNMENT_OFFSET),
            (FieldConstants.GET_TOWER_POSTS().get(2).getY() - AutoConstants.TOWER_ALIGNMENT_OFFSET_Y),
            FieldConstants.GET_TOWER_POSTS().get(2).getRotation()
        );
        Pose2d postRight = PoseCalculations.behindTower(swerve.getPose())? new Pose2d(
            (FieldConstants.GET_TOWER_POSTS().get(1).getX() - AutoConstants.TOWER_ALIGNMENT_OFFSET),
            (FieldConstants.GET_TOWER_POSTS().get(1).getY() + AutoConstants.TOWER_ALIGNMENT_OFFSET_Y),
            FieldConstants.GET_TOWER_POSTS().get(1).getRotation()
        ) : new Pose2d(
            (FieldConstants.GET_TOWER_POSTS().get(1).getX() + AutoConstants.TOWER_ALIGNMENT_OFFSET),
            (FieldConstants.GET_TOWER_POSTS().get(1).getY() + AutoConstants.TOWER_ALIGNMENT_OFFSET_Y),
            FieldConstants.GET_TOWER_POSTS().get(1).getRotation()
        );
        Pose2d node;

        switch(alignmentIndex) {
            case 1:
                node = postLeft;
            break;
            case -1:
                node = postRight;
            break;
            default:
                node = towerCenter;
        }

        double distance = DriveConstants.FULL_ROBOT_LENGTH_METERS / 2;
        Pose2d desiredPose = PoseCalculations.getPoseWithDistance(node, distance);

        ChassisSpeeds autoSpeeds = getProfiledAutoSpeeds(desiredPose);

        return autoSpeeds;
    }

    public ChassisSpeeds getAllianceLineAlignmentSpeeds() {
        Pose2d node = PoseCalculations.nearest(swerve.getPose(), FieldConstants.ALLIANCE_LINE_LIST);

        double distance = DriveConstants.FULL_ROBOT_LENGTH_METERS / 2;
        Pose2d desiredPose = PoseCalculations.getPoseWithDistance(node, distance);

        ChassisSpeeds autoSpeeds = getProfiledAutoSpeeds(desiredPose);

        return autoSpeeds;
    }

    public void updateIndex(int increment) {
        resetHDC();
        resetProfile();
        alignmentIndex = increment;
    }

    public Command updateIndexCommand(int increment) {
        return Commands.runOnce(() -> updateIndex(increment));
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds, boolean resetAlignment) {
        return 
            Commands.sequence(
                resetHDCCommand(),
                Commands.runOnce(() -> this.alignmentMode = mode),
                swerve.getDriveCommand(
                () -> 
                    normalizeChassisSpeeds(
                        autoSpeeds.get(), 
                        controllerSpeeds.get()
                    ), 
                    () -> false
                )
            ).finallyDo(() -> {
                resetProfile();
                resetHDC();
                swerve.setDesiredPose(Pose2d.kZero);
                if (resetAlignment) {
                    this.alignmentMode = AlignmentMode.NONE;
                    this.alignmentIndex = 0;
                }
            });
    }

    public Command autoAlignmentCommand(AlignmentMode mode, Supplier<ChassisSpeeds> autoSpeeds, boolean resetAlignment) {
        return 
            Commands.sequence(
                resetHDCCommand(),
                Commands.runOnce(() -> this.alignmentMode = mode),
                swerve.getDriveCommand(
                    () -> normalizeChassisSpeeds(autoSpeeds.get()),
                    () -> false
                )
            ).finallyDo(() -> {
                resetHDC();
                resetProfile();
                swerve.setDesiredPose(Pose2d.kZero);
                if (resetAlignment) {
                    this.alignmentMode = AlignmentMode.NONE;
                    this.alignmentIndex = 0;
                }
            });
    }

    public Command autoBumpRotationCommand(Pose2d robotPos) {
        Pose2d desiredPos = new Pose2d(robotPos.getX(), robotPos.getY(), PoseCalculations.getClosestBumpRotation(robotPos));
        return Commands.runOnce(() -> this.alignmentMode = AlignmentMode.BUMP).andThen(Commands.run(() -> swerve.setDesiredPose(desiredPos)));
    }

    public Command towerAlignmentCommand(Pose2d robotPos) {
        return
            autoAlignmentCommand(
                AlignmentMode.TOWER, 
                this::getTowerAlignmentSpeeds,
                true
            );
    }

    public Command startingLineAlignmentCommand(Pose2d robotPos) {
        return autoAlignmentCommand(AlignmentMode.STARTING_LINE, this::getAllianceLineAlignmentSpeeds, true);
    }
}
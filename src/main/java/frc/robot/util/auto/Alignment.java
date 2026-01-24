package frc.robot.util.auto;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;

public class Alignment {

    private final Swerve swerve;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentMode")
    private AlignmentMode alignmentMode = AlignmentMode.NONE;
    @AutoLogOutput (key = "Subsystems/Swerve/AlignmentIndex")
    private int alignmentIndex = -1;

    private State xySetpoint = new State(0, 0);
    private State targetXY = new State(0, 0);

    private Pose2d profileStartingPose = Pose2d.kZero;

    // private LoggedTunableNumber translationVelocity = new LoggedTunableNumber("Alignment/TranslationVelocity", AutoConstants.HDC_XY_VELOCITY);
    // private LoggedTunableNumber translationAcceleration = new LoggedTunableNumber("Alignment/TranslationAcceleration", AutoConstants.HDC_XY_ACCELERATION);

    public Alignment(Swerve swerve) {
        this.swerve = swerve;

        // translationVelocity.onChanged().or(translationAcceleration.onChanged())
        //     .onTrue(Commands.runOnce(() -> {
        //         xyProfile = new TrapezoidProfile(new Constraints(translationVelocity.get(), translationAcceleration.get()));
        //     }).ignoringDisable(true));
    }

    //edit when calc poses are done
    public enum AlignmentMode {
        NONE
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


    public ChassisSpeeds getControllerSpeeds(double controllerX, double controllerY) {
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                controllerY * (Robot.isRedAlliance() ? -1 : 1), 
                controllerX * (Robot.isRedAlliance() ? 1 : -1), 
                0, 
                swerve.getPose().getRotation()
            );
    }

    public AlignmentMode getAlignmentMode() {
        return alignmentMode;
    }

}

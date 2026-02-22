package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.hood.HoodIOKraken;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.shooter.turret.TurretIOKraken;
import frc.robot.util.Constants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.ShotCalculations;
import frc.robot.util.calc.ShotCalculations.ShootingParameters;

public class Shooter extends SubsystemBase {

    @AutoLogOutput(key = "Susbsytems/Shooter/TrackTarget")
    private boolean trackTarget = true;
    private int passIndex = 0;

    private Turret turret;
    private Hood hood;
    private Flywheel flywheel;

    private Translation3d targetLocation;
    @AutoLogOutput(key = "Subsystems/Shooter/Target")
    private String targetName = "None";

    private Supplier<Pose2d> robotRelativePose;
    private Supplier<ChassisSpeeds> robotRelativeVelocity;

    public Shooter(Supplier<Pose2d> robotRelativePose, Supplier<ChassisSpeeds> robotRelativeVelocity, Turret turret, Flywheel flywheel, Hood hood) {
        this.robotRelativePose = robotRelativePose;
        this.robotRelativeVelocity = robotRelativeVelocity;

        targetLocation = FieldConstants.SHOT_TARGETS.get("Hub");

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
    }

    @Override
    public void periodic() {
        if (PoseCalculations.inTrench(robotRelativePose.get())) {
            hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS);
        }
        else if (trackTarget) {
            updateTarget(robotRelativePose.get());
            ShootingParameters shotParams = ShotCalculations.getParameters(robotRelativePose, robotRelativeVelocity, targetLocation, true);
            flywheel.setVelocity(shotParams.flywheelSpeed());
            hood.setPosition(shotParams.hoodAngle());     
            turret.setPosition(shotParams.turretAngle().minus(robotRelativePose.get().getRotation()).getRadians());       
        }
    }

    public boolean isTrackingTarget() {
        return trackTarget;
    }

    public void setTrackTarget(boolean trackTarget) {
        if (!trackTarget) {
            targetName = "None";
        }
        this.trackTarget = trackTarget;
    }

    public Command setTrackTargetCommand(BooleanSupplier trackTarget) {
        return Commands.runOnce(() -> setTrackTarget(trackTarget.getAsBoolean()));
    }

    /**
     * Returns true if the shooter is at the right speed and angle
     * 
     * @return whether the shooter is at the right state
     */
    public boolean shooterReady() {
        return turret.atTargetPosition() && flywheel.atTargetVelocity() && hood.atTargetPosition();
    }

    public void setPassIndex(int index) {
        passIndex = index;
    }

    public Command setPassIndexCommand(int index) {
        return Commands.runOnce(() -> setPassIndex(index));
    }

    public Command setNeutralCommand() {
        return Commands.runOnce(() -> this.setNeutral());
    }

    public void setNeutral() {
        turret.setNeutral();
        hood.setNeutral();
        flywheel.setNeutral();
    }

    public void updateTarget(Pose2d robotPose) {
        targetName = PoseCalculations.getZone(robotPose) == PoseCalculations.ZONE.inAllianceZone 
                        ? "Hub" 
                        : passIndex == 0 ? "Pass Left" 
                        : "Pass Right";
        targetLocation = FieldConstants.SHOT_TARGETS.get(targetName);
    }

    public void eject() {
        flywheel.setVelocity(FlywheelConstants.EJECT_VELOCITY);
    }

    public Command ejectCommand() {
        return Commands.runOnce(() -> this.eject());
    }
}

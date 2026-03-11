package frc.robot.subsystems.superstructure.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.ShotCalculations;
import frc.robot.util.calc.ShotCalculations.ShootingParameters;

// TODO: overall the logic in here is convoluted and makes it difficult to follow branches and debug
public class Shooter extends SubsystemBase {

    @AutoLogOutput(key = "Susbsytems/Shooter/AimTurret")
    private boolean aimTurret = true;
    @AutoLogOutput(key = "Susbsytems/Shooter/TrackFlywheel")
    private boolean spinFlywheel = false;
    @AutoLogOutput(key = "Susbsytems/Shooter/AimHood")
    private boolean aimHood = true;
    private int passIndex = 0;
    private Target target;

    private ShootingParameters shotParams;

    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private Translation3d targetLocation;
    @AutoLogOutput(key = "Subsystems/Shooter/Target")
    private String targetName = "None";

    private Supplier<Pose2d> robotRelativePose;
    private Supplier<ChassisSpeeds> robotRelativeVelocity;

    public Shooter(Supplier<Pose2d> robotRelativePose, Supplier<ChassisSpeeds> robotRelativeVelocity, Turret turret, Flywheel flywheel, Hood hood) {
        this.robotRelativePose = robotRelativePose;
        this.robotRelativeVelocity = robotRelativeVelocity;

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;

        shotParams = new ShootingParameters(false, new Rotation2d(0.0), 0.0, 0.0, 0.0, 0.0);
        target = Target.HUB;
        targetLocation = FieldConstants.GET_HUB();
    }

    public enum Target {
        HUB(FieldConstants.GET_HUB()),
        PASS_LEFT(FieldConstants.GET_PASS_LEFT()),
        PASS_RIGHT(FieldConstants.GET_PASS_RIGHT()),
        NONE(new Translation3d(0.0, 0.0, 0.0));

        public Translation3d location;

        Target(Translation3d location) {
            this.location = location;
        }
    }

    @Override
    public void periodic() {
        if (partiallyTracking()) {
            updateTarget(robotRelativePose.get());
            this.setShooterStates();
        }
        else {
            target = Target.NONE;
            hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS);
            flywheel.setNeutral();
        }
    }

    // TODO: consider refactoring this functionality. when do we want partial/full tracking?
    // it looks like they are all unified, and in that case, the redundancy of three variables should be condensed to one
    public boolean partiallyTracking() {
        return aimHood || aimTurret || spinFlywheel;
    }

    public boolean fullyTracking() {
        return aimHood && aimTurret && spinFlywheel;
    }

    public boolean hoodTracking() {
        return aimHood;
    }

    public boolean turretTracking() {
        return aimTurret;
    }

    public boolean flywheelTracking() {
        return spinFlywheel;
    }

    // TODO: (2) flatten this with the periodic state checks (also try to consolidate these conditions)
    public void setShooterStates() {
        // calculate states for turret, flywheel, and hood based on robot location
        shotParams = ShotCalculations.getParameters(robotRelativePose, robotRelativeVelocity, targetLocation, target == Target.HUB);

        // set the flywheel and turret states if we want them to track a target
        if (spinFlywheel) flywheel.setSpeedRPMCommand(() -> shotParams.flywheelSpeed());
        if (aimTurret) turret.setPositionCommand(() -> shotParams.turretAngle().minus(robotRelativePose.get().getRotation()));

        // if the hood is under the trench, the hood should be down, otherwise it can be tracking the target
        if (PoseCalculations.shouldPutHoodDown(robotRelativePose.get())) {
            hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS);
        }
        else if (aimHood){
            hood.setPositionCommand(() -> shotParams.hoodAngle());
        }
    }

    public Command trackTargetCommand(BooleanSupplier aimTurret, BooleanSupplier aimHood, BooleanSupplier spinFlywheel) {
        return runOnce(() -> setTrackTarget(aimTurret.getAsBoolean(), aimHood.getAsBoolean(), spinFlywheel.getAsBoolean()));
    }

    public Command trackTargetCommand(boolean aimTurret, boolean aimHood, boolean spinFlywheel) {
        return runOnce(() -> setTrackTarget(aimTurret, aimHood, spinFlywheel));
    }

    public void setTrackTarget(boolean aimTurret, boolean aimHood, boolean spinFlywheel) {
        this.spinFlywheel = spinFlywheel;
        this.aimTurret = aimTurret;
        this.aimHood = aimHood;
    }

    public Command armShooterCommand() {
        return runOnce(() -> setTrackTarget(true, true, true));
    }

    public Command disarmShooterCommand() {
        return runOnce(() -> setTrackTarget(false, false, false));
    }

    /**
     * Returns true if the shooter is at the right speed and angle
     * 
     * @return whether the shooter is at the right state
     */
    public boolean isReady() {
        return turret.atTargetPosition() && flywheel.atTargetSpeed() && hood.atTargetPosition();
    }

    /**
     * Sets the pass index of the shooter.
     * 0 is for the left of driver station and 1 is for the right
     * 
     * @param index 
     */
    public void setPassIndex(int index) {
        passIndex = index;
    }

    public Command setPassIndexCommand(int index) {
        return runOnce(() -> setPassIndex(index));
    }

    public Command setNeutralCommand() {
        return runOnce(() -> this.setNeutral());
    }

    public void setNeutral() {
        turret.setNeutral();
        hood.setNeutral();
        flywheel.setNeutral();
    }

    // TODO: use enum to lower multiplicity of values instead of string
    public void updateTarget(Pose2d robotPose) {
        target = PoseCalculations.getZone(robotPose) == PoseCalculations.ZONE.inAllianceZone 
                        ? Target.HUB 
                        : passIndex == 0 ? Target.PASS_LEFT 
                        : Target.PASS_RIGHT;
    }

    public void eject() {
        flywheel.setSpeedRPM(FlywheelConstants.EJECT_VELOCITY);
    }

    public Command ejectCommand() {
        return runOnce(() -> this.eject());
    }

    public Command stowCommand() {
        return Commands.sequence(
                    hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS),
                    this.setNeutralCommand()
                );
    }

    public Target getTarget() {
        return target;
    }

    public boolean aimingAtHub() {
        return target == Target.HUB ? true : false;
    }
}

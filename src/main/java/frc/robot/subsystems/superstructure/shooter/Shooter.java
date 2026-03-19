package frc.robot.subsystems.superstructure.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.ShotCalculations;
import frc.robot.util.calc.ShotCalculations.ShootingParameters;

public class Shooter extends SubsystemBase {

    @AutoLogOutput(key = "Subsystems/Shooter/AimTurret")
    private boolean aimTurret = false;
    @AutoLogOutput(key = "Subsystems/Shooter/TrackFlywheel")
    private boolean spinFlywheel = false;
    @AutoLogOutput(key = "Subsystems/Shooter/AimHood")
    private boolean aimHood = false;
    @AutoLogOutput(key = "Subsystems/Shooter/PassIndex")
    private int passIndex = 0;
    @AutoLogOutput(key = "Subsystems/Shooter/Manual")
    private boolean manual = false;
    private Target target;

    private ShootingParameters shotParams;

    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private Translation3d targetLocation;

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
        HUB(FieldConstants.GET_HUB(), "Hub"),
        PASS_LEFT(FieldConstants.GET_PASS_LEFT(), "Pass Left"),
        PASS_RIGHT(FieldConstants.GET_PASS_RIGHT(), "Pass Right"),
        NONE(new Translation3d(0.0, 0.0, 0.0), "None");

        public Translation3d location;
        public String name;

        Target(Translation3d location, String name) {
            this.location = location;
            this.name = name;
        }
    }

    @Override
    public void periodic() {
        if (partiallyTracking()) {
            updateTarget(robotRelativePose.get());
            if (manual) {
                turret.setPositionCommand(TurretConstants.HUB_POSITION);
                flywheel.setSpeedRPM(FlywheelConstants.HUB_RPM);
                hood.setPositionCommand(HoodConstants.HUB_POSITION_RADIANS);
            }
            else {
                shotParams = ShotCalculations.getParameters(robotRelativePose, robotRelativeVelocity, targetLocation, target == Target.HUB);
                if (aimTurret) {
                    turret.setPositionCommand(shotParams.turretAngle().minus(robotRelativePose.get().getRotation()));
                }
                if (aimHood) {
                    hood.setPositionCommand(() -> shotParams.hoodAngle());
                }
                if (spinFlywheel) {
                    flywheel.setSpeedRPMCommand(() -> shotParams.flywheelSpeed());
                }
            }
        }
        else {
            target = Target.NONE;
        }

        Logger.recordOutput("Subsystems/Shooter/Target", target.name);
        Logger.recordOutput("Subsystems/Shooter/Turret/AtTargetPosition", turret.atTargetPosition(this::aimingAtHub));
        Logger.recordOutput("Subsystems/Shooter/Hood/AtTargetPosition", hood.atTargetPosition(this::aimingAtHub));
        Logger.recordOutput("Subsystems/Shooter/Flywheel/AtTargetSpeed", flywheel.atTargetSpeed(this::aimingAtHub));

    }

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

    public Command setShooterStates() {
        if (manual) {
            return Commands.runOnce(() -> setManualStates());
        }
        else {
            return Commands.runOnce(() -> setAutoAimStates());
        }
    }

    public Command setManualStates() {
        return Commands.sequence(
            flywheel.setSpeedRPMCommand(FlywheelConstants.HUB_RPM),
            hood.setPositionCommand(HoodConstants.HUB_POSITION_RADIANS),
            turret.setPositionCommand(Rotation2d.fromRadians(TurretConstants.MIN_ANGLE_RADIANS))
        );
    }

    // maybe delete this
    public void setAutoAimStates() {
        // calculate states for turret, flywheel, and hood based on robot location
        shotParams = ShotCalculations.getParameters(robotRelativePose, robotRelativeVelocity, targetLocation, target == Target.HUB);
        flywheel.setSpeedRPM(shotParams.flywheelSpeed());
        hood.setPosition(shotParams.hoodAngle());
        turret.setPosition(shotParams.turretAngle().minus(robotRelativePose.get().getRotation()));
        // return Commands.sequence(
        //     Commands.either(
        //         flywheel.setSpeedRPMCommand(() -> shotParams.flywheelSpeed()),
        //         Commands.none(),
        //         this::flywheelTracking
        //     ),
        //     Commands.either(
        //         hood.setPositionCommand(() -> shotParams.hoodAngle()),
        //         Commands.none(),
        //         this::hoodTracking
        //     ),
        //     Commands.either(
        //         turret.setPositionCommand(shotParams.turretAngle().minus(robotRelativePose.get().getRotation())),
        //         Commands.none(),
        //         this::turretTracking
        //     )
        // );

        // // set the flywheel and turret states if we want them to track a target
        // if (spinFlywheel) {
        //     flywheel.setCalculatedSpeedRPMCommand(() -> shotParams.flywheelSpeed());
        //     flywheel.runCalculateSpeedCommand();
        //     // flywheel.setSpeedRPMCommand(() -> shotParams.flywheelSpeed());
        // }

        // if (aimTurret) {
        //     turret.setPositionCommand(() -> shotParams.turretAngle().minus(robotRelativePose.get().getRotation()));

        // }
        // // if the hood is under the trench, the hood should be down, otherwise it can be tracking the target
        // // if (PoseCalculations.shouldPutHoodDown(robotRelativePose.get())) {
        // //     hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS);
        // // }
        // // else if (aimHood){
        //     hood.setPositionCommand(() -> shotParams.hoodAngle());
        // // }
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
        return Commands.either(
                trackTargetCommand(true, true, true),
                Commands.none(),
                this::isManual
            );
    }

    public Command disarmShooterCommand() {
        return Commands.sequence(
                    this.trackTargetCommand(true, false, false),
                    hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS),
                    flywheel.setNeutralCommand()
                );
    }

    /**
     * Returns true if the shooter is at the right speed and angle
     * 
     * @return whether the shooter is at the right state
     */
    public boolean isReadyToShoot() {
        return this.fullyTracking() 
                    && turret.atTargetPosition(this::aimingAtHub) 
                    && flywheel.atTargetSpeed(this::aimingAtHub) 
                    && hood.atTargetPosition(this::aimingAtHub);
    }

    /**
     * Sets the pass index of the shooter.
     * -1 is for the left and 1 is for the right sides of the field and 0 is for automatic
     * 
     * @param index 
     */
    public void setPassIndex(int index) {
        if (passIndex == -1) {
            passIndex = 0;
        }
        passIndex += index;

        //TODO make sure this works idk if it will probably not im stupid
        if (PoseCalculations.getZone(robotRelativePose.get()) == PoseCalculations.ZONE.inNeutralZone) {
            MathUtil.clamp(passIndex, -1, 1);
        } else {
            passIndex = 0;
        }

    }

    public void autoSetPassIndex() {
        passIndex = PoseCalculations.fieldHalf(robotRelativePose.get()) ? 1 : -1;
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

    public void updateTarget(Pose2d robotPose) {
        if (PoseCalculations.getZone(robotRelativePose.get()) == PoseCalculations.ZONE.inAllianceZone) {
            target = Target.HUB;
            return;
        } else if (passIndex == 0) {
            autoSetPassIndex();
        }

        target = passIndex == -1 ? Target.PASS_LEFT : Target.PASS_RIGHT;

        targetLocation = target == Target.HUB 
                                    ? Target.HUB.location 
                                    : target == Target.PASS_LEFT 
                                            ? Target.PASS_LEFT.location 
                                            : Target.PASS_RIGHT.location;
    }

    public void eject() {
        flywheel.setSpeedRPM(FlywheelConstants.EJECT_RPM);
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

    public void toggleManualShooting() {
        manual = !manual;
    }

    public Command toggleManualShootingCommand() {
        return runOnce(() -> this.toggleManualShooting());
    }

    public boolean isManual() {
        return manual;
    }
}

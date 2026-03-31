package frc.robot.subsystems.superstructure.shooter;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
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
import frc.robot.util.custom.ActiveConditionalCommand;

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

    private ShootingParameters shotParams = new ShootingParameters(true, new Rotation2d(), 0, 0, 0, 0);

    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private Translation3d targetLocation;

    private Supplier<Pose2d> robotRelativePose;
    private Supplier<ChassisSpeeds> robotRelativeVelocity;

    private Set<SubsystemBase> requirements;

    @AutoLogOutput (key = "Subsytems/shooter/EstimatedHoodAngle")
    private double estimatedHoodAngle = shotParams.hoodAngle();

    @AutoLogOutput (key = "Subsytems/shooter/EsitmatedFlywheelSpeed")
    private double estimatedFlywheelSpeed = shotParams.flywheelSpeed();

    public Shooter(Supplier<Pose2d> robotRelativePose, Supplier<ChassisSpeeds> robotRelativeVelocity, Turret turret, Flywheel flywheel, Hood hood) {
        this.robotRelativePose = robotRelativePose;
        this.robotRelativeVelocity = robotRelativeVelocity;

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;

        requirements = Set.of(turret, hood, flywheel, this);

        
        target = Target.HUB;
        targetLocation = FieldConstants.GET_HUB();
        calculateShotParams();

        // turret.setDefaultCommand(
        //     new ActiveConditionalCommand(
        //             turret.setPositionCommand(() -> shotParams.turretAngle()),
        //             turret.setNeutralCommand(), 
        //             () -> this.turretTracking()).alongWith(Commands.print("Turret default"))
        // );

        // flywheel.setDefaultCommand(
        //     new ActiveConditionalCommand(
        //         flywheel.setSpeedRPMCommand(() -> shotParams.flywheelSpeed()),
        //         flywheel.stopCommand(), 
        //         () -> this.flywheelTracking())
        // );

        // hood.setDefaultCommand(
        //     new ActiveConditionalCommand(
        //         hood.setPositionCommand(() -> shotParams.hoodAngle()),
        //         hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS),
        //         () -> this.hoodTracking())
        // );
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
        updateTarget(robotRelativePose.get());
        calculateShotParams();
        estimatedFlywheelSpeed = shotParams.flywheelSpeed();
        estimatedHoodAngle = shotParams.hoodAngle();
        // set turret's param in turret
    
        if (isManual()) {
            setTrackTarget(false, false, false);
            flywheel.setSpeedRPM(FlywheelConstants.HUB_RPM);
            hood.setPosition(HoodConstants.HUB_POSITION_RADIANS);
            turret.setPosition(TurretConstants.HUB_POSITION);
            // System.out.println("MANUAL");
        }
        else {
            if (turretTracking()) {
                turret.setPosition(shotParams.turretAngle());
            }
                
            if (flywheelTracking()) {
                flywheel.setSpeedRPM(shotParams.flywheelSpeed());
            }
            else {
                flywheel.stop();
            }
        
            if (hoodTracking()) {
                hood.setPosition(shotParams.hoodAngle());
            }
            else {
                hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
            }

            if (!partiallyTracking()) {
                flywheel.stop();
                hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
                target = Target.NONE;
            }
        }


        Logger.recordOutput("Subsystems/Shooter/ShotParamFlywheelSpeed", shotParams.flywheelSpeed());
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

    public void setManualStates() {
        flywheel.setSpeedRPM(FlywheelConstants.HUB_RPM);
        hood.setPosition(HoodConstants.HUB_POSITION_RADIANS);
        turret.setPosition(TurretConstants.HUB_POSITION);
    }

    public Command setManualStatesCommand() {
        return Commands.sequence(
            flywheel.setSpeedRPMCommand(FlywheelConstants.HUB_RPM),
            hood.setPositionCommand(HoodConstants.HUB_POSITION_RADIANS),
            turret.setPositionCommand(Rotation2d.fromRadians(TurretConstants.MIN_ANGLE_RADIANS))
        );
    }

    public Command trackTargetCommand(BooleanSupplier aimTurret, BooleanSupplier aimHood, BooleanSupplier spinFlywheel) {
        return runOnce(() -> setTrackTarget(aimTurret.getAsBoolean(), aimHood.getAsBoolean(), spinFlywheel.getAsBoolean()));
    }

    public Command trackTargetCommand(boolean aimTurret, boolean aimHood, boolean spinFlywheel) {
        return runOnce(() -> setTrackTarget(aimTurret, aimHood, spinFlywheel));
    }

    public void setTrackFlywheel(boolean track) {
        spinFlywheel = track;
    }

    public Command setTrackFlywheelCommand(boolean track)  {
        return runOnce(() -> setTrackFlywheel(track));
    }

    public void toggleTrackTurret() {
        aimTurret = !aimTurret;
    }

    public Command toggleTrackTurretCommand() {
        return runOnce(() -> toggleTrackTurret());
    }

    public void setTrackTarget(boolean aimTurret, boolean aimHood, boolean spinFlywheel) {
        this.spinFlywheel = spinFlywheel;
        this.aimTurret = aimTurret;
        this.aimHood = aimHood;
    }

    public Command armShooterCommand() {
        return trackTargetCommand(true, true, true);
    }

    public Command disarmShooterCommand(boolean trackTurret) {
        return Commands.sequence(
                    this.trackTargetCommand(trackTurret, false, false),
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
        passIndex = index;
    }

    public void autoSetPassIndex() {
        passIndex = PoseCalculations.fieldHalf(robotRelativePose.get()) ? 1 : -1;
    }

    public Command setPassIndexCommand(int index) {
        return runOnce(() -> setPassIndex(index));
    }

    public void setNeutral() {
        turret.setNeutral();
        hood.setNeutral();
        flywheel.setNeutral();
    }

    public Command setNeutralCommand() {
        return Commands.parallel(
                turret.setNeutralCommand(),
                hood.setNeutralCommand(),
                flywheel.setNeutralCommand()
            );
    }

    public void updateTarget(Pose2d robotPose) {
        if (PoseCalculations.inShootingZone(robotRelativePose.get())) {
            target = Target.HUB;
            targetLocation = Target.HUB.location;
            passIndex = 0;
        } 
        else {
            if (passIndex == 0) {
                autoSetPassIndex();
            }
            target = passIndex == -1 ? Target.PASS_LEFT : Target.PASS_RIGHT;
            targetLocation = target == Target.PASS_LEFT 
                                                ? Target.PASS_LEFT.location 
                                                : Target.PASS_RIGHT.location;
        }


    }

    public void calculateShotParams() {
        shotParams = ShotCalculations.getParameters(robotRelativePose, robotRelativeVelocity, targetLocation, target == Target.HUB);
    }

    public Command ejectCommand() {
        return flywheel.setSpeedRPMCommand(FlywheelConstants.EJECT_RPM);
    }

    public Command stowCommand() {
        return Commands.sequence(
                    hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS),
                    this.setNeutralCommand()
                );
    }

    public void stow() {
        hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
        this.setNeutral();
    }

    public void forceHoodDown() {
        aimHood = false;
        hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
    }

    public Command forceHoodDownCommand() {
        return runOnce(() -> forceHoodDown());
    }

    public Target getTarget() {
        return target;
    }

    public boolean aimingAtHub() {
        return target == Target.HUB ? true : false;
    }

    public void toggleManualShooting() {
        manual = !manual;
        // if (manual) {
        //     setTrackTarget(false, false, false);
        //     flywheel.setSpeedRPM(FlywheelConstants.HUB_RPM);
        //     hood.setPosition(HoodConstants.HUB_POSITION_RADIANS);
        //     turret.setPositionCommand(TurretConstants.HUB_POSITION);
        // }
        // else {
        //     flywheel.stop();
        //     hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
        // }
    }

    public Command toggleManualShootingCommand() {
        return runOnce(() -> this.toggleManualShooting());
    }

    // public Command setManualShootingCommand(boolean manual) {
    //     return Commands.runOnce(() -> this.manual = manual);
    // }

    // public Command toggleManualShootingCommand() {
    //     return Commands.parallel(
    //             this.setManualShootingCommand(true),
    //             flywheel.setSpeedRPMCommand(FlywheelConstants.HUB_RPM),
    //             hood.setPositionCommand(HoodConstants.HUB_POSITION_RADIANS),
    //             turret.setPositionCommand(TurretConstants.HUB_POSITION),
    //             Commands.print("Turret manual")
    //     ).repeatedly();
    // }

    // public Command enableManualCommand() {
    //     return Commands.parallel(
    //             this.setManualShootingCommand(true),
    //             flywheel.setSpeedRPMCommand(FlywheelConstants.HUB_RPM),
    //             hood.setPositionCommand(HoodConstants.HUB_POSITION_RADIANS),
    //             turret.setPositionCommand(TurretConstants.HUB_POSITION));
    // }

    // public Command disableManualCommand() {
    //     return Commands.parallel(
    //             this.setManualShootingCommand(false),
    //             flywheel.stopCommand(),
    //             hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS),
    //             turret.setNeutralCommand());
    // }


    public boolean isManual() {
        return manual;
    }
}

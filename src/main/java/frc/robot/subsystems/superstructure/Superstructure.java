package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.calc.ShotCalculations;
import frc.robot.util.calc.ShotCalculations.ShootingParameters;

public class Superstructure {

    private int passIndex = 1;

    public Extender extender;
    public Intake intake;
    public Climb climb;
    public Turret turret;
    public Hood hood;
    public Flywheel flywheel;
    public Spindexer spindexer;
    public Kicker kicker;

    private final Set<Subsystem> requirements;
    
    private final Supplier<Pose2d> robotPoseSupplier;

    public ClimbState targetClimbState;
    public IntakeState targetIntakeState;

    public SuperState currentState;
    public SuperState targetState;

    public SuperState IDLE;
    public SuperState INTAKE;
    public SuperState INTAKE_WHILE_SHOOT;
    public SuperState READY_SHOOTER;
    public SuperState SHOOT;
    public SuperState EJECT;
    public SuperState CLIMB_READY;
    public SuperState CLIMB_READY_WHILE_SHOOT;
    public SuperState CLIMB_UP;
    public SuperState CLIMB_UP_WHILE_SHOOT;


    public Superstructure(Extender extender, Intake intake, Climb climb, Turret turret, Hood hood, Flywheel flywheel, Spindexer spindexer, Kicker kicker, Supplier<Pose2d> robotPoseSupplier) {
        this.extender = extender;
        this.intake = intake;
        this.climb = climb;
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.spindexer = spindexer;
        this.kicker = kicker;

        this.requirements = Set.of(extender, intake, climb, turret, hood, flywheel, spindexer, kicker);

        this.robotPoseSupplier = robotPoseSupplier;

        IDLE = new SuperState("IDLE", HopperState.OFF, ClimbState.STOW, IntakeState.STOW, ShooterState.STOW);

        targetState = IDLE;

        INTAKE = new SuperState("INTAKE", HopperState.OFF, ClimbState.STOW, IntakeState.INTAKE, ShooterState.STOW);
        INTAKE_WHILE_SHOOT = new SuperState("INTAKE_WHILE_SHOOT", HopperState.FORWARD, ClimbState.STOW, IntakeState.INTAKE, ShooterState.SHOOT);
        READY_SHOOTER = new SuperState("SHOOT", HopperState.OFF, ClimbState.STOW, IntakeState.STOW, ShooterState.SHOOT);
        SHOOT = new SuperState("SHOOT", HopperState.FORWARD, ClimbState.STOW, IntakeState.STOW, ShooterState.SHOOT);
        EJECT = new SuperState("EJECT", HopperState.FORWARD, ClimbState.STOW, IntakeState.STOW, ShooterState.EJECT);
        CLIMB_READY = new SuperState("CLIMB_READY", HopperState.OFF, ClimbState.READY, IntakeState.STOW, ShooterState.STOW);
        CLIMB_READY_WHILE_SHOOT = new SuperState("CLIMB_READY_WHILE_SHOOT", HopperState.OFF, ClimbState.READY, IntakeState.STOW, ShooterState.STOW);
        CLIMB_UP = new SuperState("CLIMB_UP", HopperState.OFF, ClimbState.UP, IntakeState.STOW, ShooterState.STOW);
        CLIMB_UP_WHILE_SHOOT = new SuperState("CLIMB_UP_WHILE_SHOOT", HopperState.OFF, ClimbState.UP, IntakeState.STOW, ShooterState.STOW);
    }

    public enum ClimbState {
        STOW(ClimbConstants.STOW_POSITION_METERS, false),
        READY(ClimbConstants.MAX_DISPLACEMENT_METERS, false),
        UP(ClimbConstants.STOW_POSITION_METERS, true);

        public double climbPosition;
        public boolean slam;

        ClimbState(double climbPosition, boolean slam) {
            this.climbPosition = climbPosition;
            this.slam = slam;
        }
    }

    public enum HopperState {
        FORWARD(KickerConstants.FORWARD_PERCENT, SpindexerConstants.COUNTERCLOCKWISE_PERCENT),
        REVERSE(KickerConstants.REVERSE_PERCENT, SpindexerConstants.CLOCKWISE_PERCENT),
        OFF(KickerConstants.IDLE_PERCENT, SpindexerConstants.IDLE_PERCENT);

        public double kickerPercent;
        public double spindexerPercent;
    
        HopperState(double kickerPercent, double spindexerPerncent) {
            this.kickerPercent = kickerPercent;
            this.spindexerPercent = spindexerPercent;
        }
    }

    public enum ShooterState {
        STOW,
        EJECT,
        SHOOT;
    }

    public enum IntakeState {
        STOW(ExtenderConstants.STOW_POSITION_RADIANS, 0.0),
        INTAKE(ExtenderConstants.INTAKE_POSITION_RADIANS, IntakeConstants.INTAKE_PERCENT);

        public double extenderPosition;
        public double intakePercent;

        IntakeState(double extenderPosition, double intakePercent) {
            this.extenderPosition = extenderPosition;
            this.intakePercent = intakePercent;
        }
    }

    public Command setSuperState(SuperState targetState, Swerve swerve) {
        return Commands.runOnce(() -> this.targetState = targetState).alongWith(
            Commands.parallel(
                fixIntakeAndClimb(targetState.intakeState, targetState.climbState),
                setHopperState(targetState.hopperState),
                setShooterState(targetState.shooterState, swerve)
            ));
    }

    // none of our superstates have both of these subsystems active at the same time 
    // because that would create illegal extension over the frame perimeter in multiple directions
    // so whichever one wants to stow, we stow that first
    public Command fixIntakeAndClimb(IntakeState intakeState, ClimbState climbState) {
        return Commands.either(
            Commands.sequence(setIntakeState(intakeState), setClimbState(climbState)),
            Commands.sequence(setClimbState(climbState), setIntakeState(intakeState)),
            () -> climbState != ClimbState.STOW 
        );
    }

    public Command setIntakeState(IntakeState state) {
        return Commands.runOnce(() -> targetIntakeState = state)
                .alongWith(intake.setPercentCommand(() -> state.intakePercent));
    }

    public Command setClimbState(ClimbState state) {
        return Commands.runOnce(() -> targetClimbState = state)
            .alongWith(climb.setPositionCommand(() -> state.climbPosition, () -> state.slam));
    }

    public Command setHopperState(HopperState state) {
        return Commands.sequence(kicker.setPercentCommand(() -> state.kickerPercent),
                                    spindexer.setPercentCommand(() -> state.spindexerPercent));
    }

    public Command setShooterState(ShooterState state, Swerve swerve) {
        if (state == ShooterState.STOW) {
            return stowShooter();
        }
        else if (state == ShooterState.EJECT) {
            return flywheel.setVelocityCommand(FlywheelConstants.EJECT_VELOCITY);
        }
        else {
            ShootingParameters shotParam = ShotCalculations.getParameters(swerve);
            
            return Commands.sequence(
                flywheel.setVelocityCommand(shotParam.flywheelSpeed()),
                Commands.waitSeconds(0.5),
                hood.setPositionCommand(shotParam.hoodAngle()),
                turret.setPositionCommand(shotParam.turretAngle().getRadians()));
        }
    }

    public Command aimShooter(boolean aim) {
        return Commands.sequence(
            flywheel.setTrackTargetCommand(aim),
            Commands.waitSeconds(FlywheelConstants.WAIT_TIME_SECONDS),
            turret.setTrackTargetCommand(aim),
            hood.setTrackTargetCommand(aim)
        );
    }

    /**
     * Returns true if the shooter is at the right speed and angle
     * @return whether the shooter is at the right state
     */
    public boolean shooterReady() {
        if () {
            return true;
        }
        else {
            return false;
        }
    }

    public void updatePassIndex(int index) {
        ShotCalculations.setPassIndex(index);
    }

    public Command updatePassIndexCommand(int index) {
        return Commands.runOnce(() -> updatePassIndex(index));
    }

    public Command stowShooter() {
        return Commands.parallel(
            flywheel.setNeutralCommand(),
            hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS));
    }
}

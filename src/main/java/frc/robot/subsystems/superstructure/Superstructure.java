package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure.HopperState;
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

    private int passIndex = 0;
    private boolean shooterIsTracking = true;

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
    public HopperState targetHopperState;
    public ShooterState targetShooterState;


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

        targetClimbState = ClimbState.STOW;
        targetIntakeState = IntakeState.STOW;
        targetHopperState = HopperState.OFF;
        targetShooterState = ShooterState.STOW;

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
        STOW(false),
        SHOOT(true);

        public boolean track;

        ShooterState(boolean track) {
            this.track = track;
        }
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

    // none of our superstates have both of these subsystems active at the same time 
    // because that would create illegal extension over the frame perimeter in multiple directions
    // so whichever one wants to stow, we stow that first
    // we will use this if we have an l3
    public Command fixIntakeAndClimb(IntakeState intakeState, ClimbState climbState) {
        return Commands.either(
            Commands.sequence(setIntakeState(intakeState), setClimbState(climbState)),
            Commands.sequence(setClimbState(climbState), setIntakeState(intakeState)),
            () -> climbState != ClimbState.STOW 
        );
    }

    public Command setIntakeState(IntakeState state) {
        return Commands.runOnce(() -> targetIntakeState = state)
                .alongWith(Commands.sequence(extender.setPositionCommand(state.extenderPosition), 
                intake.setPercentCommand(() -> state.intakePercent)));
    }

    public Command setClimbState(ClimbState state) {
        return Commands.runOnce(() -> targetClimbState = state)
            .alongWith(climb.setPositionCommand(() -> state.climbPosition, () -> state.slam));
    }

    public Command setHopperState(HopperState state) {
        return Commands.sequence(kicker.setPercentCommand(() -> state.kickerPercent),
                                    spindexer.setPercentCommand(() -> state.spindexerPercent));
    }

    public Command setShooterState(ShooterState state) {
        return state == ShooterState.SHOOT ? aimShooterCommand() : stowShooterCommand();
    }

    // if we are tracking a target and our shooter is ready, we will accept the request to outtake
    // if we are not tracking a target, we will assume we are trying to eject
    // if we are tracking and our shooter is not ready, then we will do nothing
    public Command outtakeCommand() {
        if (shooterIsTracking && shooterReady()) {
            return Commands.parallel(
                spindexer.setPercentCommand(HopperState.FORWARD.spindexerPercent),
                kicker.setPercentCommand(HopperState.FORWARD.kickerPercent)
            );
        }
        else if (!shooterIsTracking) {
            return Commands.sequence(
                flywheel.setVelocityCommand(FlywheelConstants.EJECT_VELOCITY),
                Commands.waitSeconds(FlywheelConstants.EJECT_WAIT_TIME_SECONDS),
                spindexer.setPercentCommand(HopperState.FORWARD.spindexerPercent),
                kicker.setPercentCommand(HopperState.FORWARD.kickerPercent)
            );
        }
        else {
            return Commands.none();
        }
    }

    public Command aimShooterCommand() {
        return Commands.runOnce(() -> shooterIsTracking = true).alongWith(
            Commands.sequence(
            flywheel.setTrackTargetCommand(true),
            Commands.waitSeconds(FlywheelConstants.SHOOT_WAIT_TIME_SECONDS),
            turret.setTrackTargetCommand(true),
            hood.setTrackTargetCommand(true)
            ));
    }

    /**
     * Returns true if the shooter is at the right speed and angle
     * 
     * @return whether the shooter is at the right state
     */
    public boolean shooterReady() {
        return turret.atTargetPosition() && flywheel.atTargetVelocity() && hood.atTargetPosition();
    }

    public void updatePassIndex(int index) {
        ShotCalculations.setPassIndex(index);
    }

    public Command updatePassIndexCommand(int index) {
        return Commands.runOnce(() -> updatePassIndex(index));
    }

    public Command stowShooterCommand() {
        return Commands.parallel(
            flywheel.setTrackTargetCommand(false),
            hood.setTrackTargetCommand(false),
            turret.setTrackTargetCommand(false),
            flywheel.setNeutralCommand(),
            hood.setPositionCommand(HoodConstants.STOW_POSITION_RADIANS));
    }
}

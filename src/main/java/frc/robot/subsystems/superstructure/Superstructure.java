package frc.robot.subsystems.superstructure;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.Constants.SpindexerConstants;

public class Superstructure {

    // TODO: make these final (you'll see why)
    private final Extender extender;
    private final Intake intake;
    private final Climb climb;
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final Kicker kicker;

    private final Set<Subsystem> requirements;

    public Superstructure(Extender extender, Intake intake, Climb climb, Shooter shooter, Spindexer spindexer, Kicker kicker) {
        this.extender = extender;
        this.intake = intake;
        this.climb = climb;
        this.shooter = shooter;
        this.kicker = kicker;
        this.spindexer = spindexer;

        this.requirements = Set.of(extender, intake, climb, shooter, spindexer, kicker);
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
    
        HopperState(double kickerPercent, double spindexerPercent) {
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
        NO_SPIN(ExtenderConstants.INTAKE_POSITION_RADIANS, 0.0),
        INTAKE(ExtenderConstants.INTAKE_POSITION_RADIANS, IntakeConstants.INTAKE_PERCENT),
        HALF_STOW(ExtenderConstants.HALF_STOW_RADIANS, 0.0);

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
        return Commands.sequence(intake.setPercentCommand(() -> state.intakePercent),
                            extender.setPositionCommand(() -> state.extenderPosition));
    }

    public Command setClimbState(ClimbState state) {
        return climb.setPositionCommand(() -> state.climbPosition, () -> state.slam);
    }

    public Command setHopperState(HopperState state) {
        return Commands.parallel(kicker.setPercentCommand(() -> state.kickerPercent),
                                    spindexer.setPercentCommand(() -> state.spindexerPercent));
    }

    // if we are tracking a target and our shooter is ready, we will accept the request to outtake
    // if we are tracking and our shooter is not ready, then we will do nothing
    // TODO: this logic is only ran once on startup because it is not lazily evaluated like Commands.either() or similar would do
    // i.e. return commands.either(true (the parallel), commands.either(elseif, else, elseif condition), first if condition)
    public Command fireCommand() {
        return Commands.either(
            Commands.parallel(
                spindexer.setPercentCommand(HopperState.FORWARD.spindexerPercent),
                kicker.setPercentCommand(HopperState.FORWARD.kickerPercent),
                extender.feedShooterCommand().until(() -> kicker.emptyHopper())),
            Commands.either(
                shooter.trackTargetCommand(true, true, true),
                Commands.none(), 
                () -> !shooter.fullyTracking()),
            () -> (shooter.fullyTracking() && shooter.isReady())
        );
    }

    public Command ceaseFireCommand() {
        return Commands.parallel(
                    shooter.disarmShooterCommand(),
                    shooter.stowCommand(),
                    setHopperState(HopperState.OFF)
                );
    }

    public Command fullStowCommand() {
        return Commands.sequence(
            shooter.stowCommand(),
            setClimbState(ClimbState.STOW),
            setHopperState(HopperState.OFF),
            setIntakeState(IntakeState.STOW)
        );
    }
}

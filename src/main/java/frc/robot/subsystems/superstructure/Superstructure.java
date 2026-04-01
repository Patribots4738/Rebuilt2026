package frc.robot.subsystems.superstructure;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

    private final Extender extender;
    private final Intake intake;
    // private final Climb climb;
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final Kicker kicker;

    private final Set<Subsystem> requirements;

    public Superstructure(Extender extender, Intake intake, Shooter shooter, Spindexer spindexer, Kicker kicker) {
        this.extender = extender;
        this.intake = intake;
        // this.climb = climb;
        this.shooter = shooter;
        this.kicker = kicker;
        this.spindexer = spindexer;

        this.requirements = Set.of(extender, intake, shooter, spindexer, kicker);
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
        HALF_STOW(ExtenderConstants.HALF_STOW_RADIANS, 0.0),
        HALF_STOW_WITH_ROLLERS(ExtenderConstants.HALF_STOW_RADIANS, IntakeConstants.INTAKE_PERCENT),
        WIGGLE(ExtenderConstants.WIGGLE_RADIANS, IntakeConstants.INTAKE_PERCENT);

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
    // public Command fixIntakeAndClimb(IntakeState intakeState, ClimbState climbState) {
    //     return Commands.either(
    //         Commands.sequence(setIntakeState(intakeState), setClimbState(climbState)),
    //         Commands.sequence(setClimbState(climbState), setIntakeState(intakeState)),
    //         () -> climbState != ClimbState.STOW 
    //     );
    // }

    public Command setIntakeState(IntakeState state) {
        return Commands.sequence(intake.setPercentCommand(() -> state.intakePercent),
                            extender.setPositionCommand(() -> state.extenderPosition));
    }

    // public Command setClimbState(ClimbState state) {
    //     return climb.setPositionCommand(() -> state.climbPosition, () -> state.slam);
    // }

    public Command setHopperState(HopperState state) {
        return Commands.parallel(kicker.setPercentCommand(() -> state.kickerPercent),
                                    spindexer.setPercentCommand(() -> state.spindexerPercent));
    }

    // if we are tracking a target and our shooter is ready, we will accept the request to outtake
    // if we are tracking and our shooter is not ready, then we will do nothing
    public Command fireCommand() {
        return shooter.armShooterCommand()
            .andThen(Commands.waitUntil(() -> shooter.isReadyToShoot()).andThen(
                Commands.sequence(
                    spindexer.setPercentCommand(HopperState.FORWARD.spindexerPercent),
                    kicker.setPercentCommand(HopperState.FORWARD.kickerPercent),
                    intake.setPercentCommand(0.0),
                    extender.feedShooterCommand())));
    }

    public Command fireNoFeedShooterCommand() {
        return shooter.armShooterCommand()
            .andThen(Commands.waitUntil(() -> shooter.isReadyToShoot()).andThen(
                Commands.sequence(
                    spindexer.setPercentCommand(HopperState.FORWARD.spindexerPercent),
                    kicker.setPercentCommand(HopperState.FORWARD.kickerPercent))));
    }

    public Command ceaseFireCommand() {
        return Commands.parallel(
                    shooter.disarmShooterCommand(true),
                    spindexer.setPercentCommand(0.0),
                    kicker.setPercentCommand(0.0),
                    intake.setPercentCommand(IntakeConstants.INTAKE_PERCENT),
                    extender.setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS)
                );
    }

    public Command fullStowCommand() {
        return Commands.sequence(
            shooter.stowCommand(),
            // setClimbState(ClimbState.STOW),
            setHopperState(HopperState.OFF),
            setIntakeState(IntakeState.STOW)
        );
    }

    public Command stowClimbAndIntake() {
        return Commands.parallel(
            setIntakeState(IntakeState.STOW)
            // ,
            // setClimbState(ClimbState.STOW)
        );
    }

    public Command toggleIntakeCommand() {
        return Commands.either(
// extender.setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS).alongWith(intake.setPercentCommand(IntakeConstants.INTAKE_PERCENT)),
//             extender.setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS).alongWith(intake.setPercentCommand(0.0)),
            setIntakeState(IntakeState.INTAKE),
            setIntakeState(IntakeState.NO_SPIN),
            () -> (intake.getPercentOutput() == 0.0)
        );
    }

    public Command killCommand() {
        return Commands.parallel(
            shooter.setNeutralCommand(),
            // climb.setNeutralCommand(),
            extender.setNeutralCommand(),
            intake.setNeutralCommand(),
            spindexer.setNeutralCommand(),
            kicker.setNeutralCommand()
        );
    }
    public void printing(){
        System.out.println("This line ran");
    }
    public Command printCommand(){
        return Commands.runOnce(()-> printing());
    }
}

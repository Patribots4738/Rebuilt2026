package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.util.Constants;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.calc.ShotCalculations;

public class Superstructure {

    private int passIncrement = 1;

    public Extender extender;
    public Intake intake;
    public Climb climb;
    public Turret turret;
    public Hood hood;
    public Spindexer spindexer;
    public Kicker kicker;

    private final Set<Subsystem> requirements;
    
    private final Supplier<Pose2d> robotPoseSupplier;

    public ClimbState targetClimbState;
    public ShooterState targetShooterState;
    public HopperState targetHopperState;
    public IntakeState targetIntakeState;

    public ClimbState currentClimbState;
    public ShooterState currentShooterState;
    public HopperState currentHopperState;
    public IntakeState currentIntakeState;


    public Superstructure(Extender extender, Intake intake, Climb climb, Turret turret, Hood hood, Spindexer spindexer, Kicker kicker, Supplier<Pose2d> robotPoseSupplier) {
        this.extender = extender;
        this.intake = intake;
        this.climb = climb;
        this.turret = turret;
        this.hood = hood;
        this.spindexer = spindexer;
        this.kicker = kicker;

        this.requirements = Set.of(extender, intake, climb, turret, hood, spindexer, kicker);

        this.robotPoseSupplier = robotPoseSupplier;
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

    public Command setIntakeState(IntakeState state) {
        return Commands.runOnce(() -> targetIntakeState = state)
                .alongWith(intake.setPercentCommand(() -> state.intakePercent));
    }

    public Command setClimbState(ClimbState state) {
        return Commands.runOnce(() -> targetClimbState = state)
            .alongWith(climb.setPositionCommand(() -> state.climbPosition, () -> state.slam));
    }

    public Command setHopperState(HopperState state) {
        return Commands.runOnce(() -> targetHopperState = state)
                .alongWith(
                    Commands.sequence(kicker.setPercentCommand(() -> state.kickerPercent),
                                        spindexer.setPercentCommand(() -> state.spindexerPercent)));
    }

    public Command ejectCommand() {
        return Commands.parallel();
    }

    public Command shootCommand() {
        return Command.run(setPercentCommand(()-> ));
    }

    public Command updatePassIndexCommand(int increment) {
        
    }
}

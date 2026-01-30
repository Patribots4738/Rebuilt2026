package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.superstructure.Superstructure.HopperState;
import frc.robot.subsystems.superstructure.Superstructure.ShooterState;
import frc.robot.subsystems.superstructure.Superstructure.IntakeState;
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;

public class SuperState {

    public final String key;
    public final HopperState hopperState;
    public final ClimbState climbState;
    public final IntakeState intakeState;
    public final ShooterState shooterState;

    public SuperState(String key, HopperState hopperState, ClimbState climbState, IntakeState intakeState, ShooterState shooterState) {
        this.key = key;
        this.hopperState = hopperState;
        this.climbState = climbState;
        this.intakeState = intakeState;
        this.shooterState = shooterState;
    }

    public SuperState(String key, HopperState hopperState, IntakeState intakeState, ShooterState shooterState) {
        this(key, hopperState, ClimbState.STOW, intakeState, shooterState);
    }
}

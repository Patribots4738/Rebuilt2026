// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.util.Constants.IntakeConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {
	
	private final FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	@AutoLogOutput (key = "Subsystems/Flywheel/EmptyHopper")
	private boolean emptyHopper;

	private Debouncer emptyHopperDebouncer = new Debouncer(0.5);
	
	private double percentOutput = 0.0;
    @AutoLogOutput (key = "Subsystems/Intake/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
	
	private final SysIdRoutine flywheelRoutine;
	
	// private LoggedTunableNumber fastVelocity = new LoggedTunableNumber("Intake/FastProfile/Velocity", IntakeConstants.FAST_VELOCITY);
    // private LoggedTunableNumber fastAcceleration = new LoggedTunableNumber("Intake/FastProfile/Acceleration", IntakeConstants.FAST_ACCELERATION);
    // private LoggedTunableNumber slowVelocity = new LoggedTunableNumber("Intake/SlowProfile/Velocity", IntakeConstants.SLOW_VELOCITY);
    // private LoggedTunableNumber slowAcceleration = new LoggedTunableNumber("Intake/SlowProfile/Acceleration", IntakeConstants.SLOW_ACCELERATION);

    // private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Intake/BrakeMotor", IntakeConstants.BRAKE_MOTOR);
    
    public Flywheel(FlywheelIO io) {
        this.io = io;
	    this.flywheelRoutine = new SysIdRoutine(
			    new SysIdRoutine.Config(
					    Volts.per(Seconds).of(1),
					    Volts.of(7),
					    Seconds.of(10)
			    ),
			    new SysIdRoutine.Mechanism(
					    (Voltage volts) -> io.setVoltage(volts.in(Volts)),
					    log -> log.motor("flywheel")
							    .voltage(Volts.of(inputs.appliedOutputVolts))
							    .angularPosition(Rotations.of(inputs.positionRotations))
							    .angularVelocity(RotationsPerSecond.of(inputs.velocityRotationsPerSec)),
					    this
			    )
	    );

        // fastVelocity.onChanged().or(fastAcceleration.onChanged()).onTrue(Commands.runOnce(() -> fastProfile = new TrapezoidProfile(new Constraints(fastVelocity.get(), fastAcceleration.get()))).ignoringDisable(true));
        // slowVelocity.onChanged().or(slowAcceleration.onChanged()).onTrue(Commands.runOnce(() -> slowProfile = new TrapezoidProfile(new Constraints(slowVelocity.get(), slowAcceleration.get()))).ignoringDisable(true));

        // IntakeConstants.LOGGED_GAINS.onChanged(Commands.runOnce(() -> io.setGains(IntakeConstants.LOGGED_GAINS.get())).ignoringDisable(true));
        // brakeMotor.onChanged(Commands.runOnce(() -> io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
		updateEmptyHopper();

        if (shouldRunSetpoint) {
            io.setPercent(percentOutput);
        } else {
            io.setNeutral();
        }

    }

    public void setPercent(double percent) {
        percentOutput = percent;
        shouldRunSetpoint = true;
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

	public void updateEmptyHopper() {
		emptyHopper = emptyHopperDebouncer.calculate(!(inputs.isRangeTripped));
	}

	public boolean emptyHopper() {
		return emptyHopper;
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return flywheelRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return flywheelRoutine.dynamic(direction);
	}
}
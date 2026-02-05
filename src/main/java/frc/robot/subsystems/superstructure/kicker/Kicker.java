// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Constants.KickerConstants;
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

import static edu.wpi.first.units.Units.*;

public class Kicker extends SubsystemBase {
	
	private final KickerIO io;
	private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
	
	private final State targetState = new State(0, 0);
	private final State setpoint = new State(0, 0);
	@AutoLogOutput (key = "Subsystems/Kicker/ShouldRunSetpoint")
	private boolean shouldRunSetpoint = false;
	@AutoLogOutput (key = "Subsystems/Kicker/ShouldRunFastProfile")
	private boolean shouldRunFast = true;
    
    private final SysIdRoutine kickerRoutine;
    
	public Kicker(KickerIO io) {
		this.io = io;
        this.kickerRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
                        log -> log.motor("kicker")
                                .voltage(Volts.of(inputs.motorAppliedOutputVolts))
                                .linearVelocity(MetersPerSecond.of(inputs.motorVelocityMetersPerSecond)),
                        this
                )
        );
	}
	
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("SubsystemInputs/Kicker", inputs);
		
		Logger.recordOutput("Subsystems/Kicker/AtTargetPosition", atTargetPosition());
		Logger.recordOutput("Subsystems/Kicker/TargetPosition", targetState.position);
		
		if (shouldRunSetpoint) {
			io.setPosition(setpoint.position);
		} else {
			setpoint.position = inputs.motorPositionMeters;
			setpoint.velocity = inputs.motorVelocityMetersPerSecond;
			io.setNeutral();
		}
	}
	
	public void setPosition(double position, boolean runFast) {
		double clampedPosition = MathUtil.clamp(position, 0, KickerConstants.MAX_DISPLACEMENT_METERS);
		if (clampedPosition != targetState.position) {
			setpoint.position = inputs.motorPositionMeters;
			setpoint.velocity = inputs.motorVelocityMetersPerSecond;
		}
		targetState.position = clampedPosition;
		shouldRunSetpoint = true;
		shouldRunFast = runFast;
	}
	
	public void setNeutral() {
		shouldRunSetpoint = false;
	}
	
	public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier runFastSupplier) {
		return runOnce(() -> setPosition(positionSupplier.getAsDouble(), runFastSupplier.getAsBoolean())).andThen(Commands.waitUntil(this::atTargetPosition));
	}
	
	public Command setPositionCommand(double position, BooleanSupplier runFastSupplier) {
		return setPositionCommand(() -> position, runFastSupplier);
	}
	
	public Command setNeutralCommand() {
		return runOnce(this::setNeutral);
	}
	
	public Command resetEncodersCommand() {
		return runOnce(() -> io.resetEncoders(0));
	}
	
	public boolean atPosition(double position) {
		return MathUtil.isNear(position, inputs.motorPositionMeters, KickerConstants.DEADBAND_RADIANS);
	}
	
	public boolean atTargetPosition() {
		return atPosition(targetState.position);
	}
	
	public double getPosition() {
		return inputs.motorPositionMeters;
	}
	
	public double getCharacterizationVelocity() {
		return inputs.motorVelocityMetersPerSecond / KickerConstants.VELOCITY_CONVERSION_FACTOR;
	}
	
	public double getVelocity() {
		return (inputs.motorVelocityMetersPerSecond);
	}
	
	public void runCharacterization(double input) {
		io.runCharacterization(input);
	}
	
	public double getVoltage() {
		return inputs.motorAppliedOutputVolts;
	}
    
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return kickerRoutine.quasistatic(direction);
    }
    
    public Command dynamic(SysIdRoutine.Direction direction) {
        return kickerRoutine.dynamic(direction);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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

public class Kicker extends SubsystemBase {
	
	private final KickerIO io;
	private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
	
	private final State targetState = new State(0, 0);
	private final State setPoint = new State(0, 0);
	@AutoLogOutput (key = "Subsystems/Kicker/ShouldRunSetpoint")
	private boolean shouldRunSetpoint = false;
	private double percentOutput;

	@AutoLogOutput (key = "Subsystems/Kicker/emptyHopper")
	private boolean emptyHopper = false;

	private Debouncer emptyHopperDebouncer;

	public Kicker(KickerIO io) {
		this.io = io;
		emptyHopperDebouncer = new Debouncer(KickerConstants.DEBOUNCER_TIME, DebounceType.kBoth);
	}
	
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		updateEmptyHopper();
	}
	
	public void setPercent(double percent) {
		percentOutput = percent;
		shouldRunSetpoint = true;
	}
	
	public void setNeutral() {
		shouldRunSetpoint = false;
	}
	
	public Command setPercentCommand(DoubleSupplier positionSupplier) {
		return runOnce(() -> setPercent(positionSupplier.getAsDouble()));
	}
	
	public Command setPercentCommand(double percent) {
		return setPercentCommand(() -> percent);
	}
	
	public Command setNeutralCommand() {
		return runOnce(this::setNeutral);
	}
	
	public Command resetEncodersCommand() {
		return runOnce(() -> io.resetEncoders(0));
	}
	
	public void runCharacterization(double input) {
		io.runCharacterization(input);
	}

	public void updateEmptyHopper() {
		emptyHopper = emptyHopperDebouncer.calculate(!(inputs.isRangeTripped));
	}
	
	public SysIdRoutine getSysIdRoutine() {
		return new SysIdRoutine(
				new SysIdRoutine.Config(
						// Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
						Volts.of(0.5).per(Second),
						null,
						Seconds.of(2),
						(state) -> Logger.recordOutput("KickerSysIdState", state.toString())
				),
				new SysIdRoutine.Mechanism(
						(voltage) -> io.runCharacterization(voltage.in(Volts)),
						null,
						this
				)
		);
	}
	
	public Command sysIdQuasistaticForward() {
		return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
	}
	
	public Command sysIdQuasistaticReverse() {
		return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse);
	}
	
	public Command sysIdDynamicForward() {
		return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
	}
	
	public Command sysIdDynamicReverse() {
		return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse);
	}
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.util.Constants.HoodConstants;
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

public class Kicker extends SubsystemBase {
	
	private final KickerIO io;
	private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

	@AutoLogOutput (key = "Subsystems/Kicker/EmptyHopper")
	private boolean emptyHopper;

	private Debouncer emptyHopperDebouncer = new Debouncer(0.5);
	
	private double percentOutput = 0.0;
    @AutoLogOutput (key = "Subsystems/Intake/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
	
    private double testPercent;

	// private LoggedTunableNumber fastVelocity = new LoggedTunableNumber("Intake/FastProfile/Velocity", IntakeConstants.FAST_VELOCITY);
    // private LoggedTunableNumber fastAcceleration = new LoggedTunableNumber("Intake/FastProfile/Acceleration", IntakeConstants.FAST_ACCELERATION);
    // private LoggedTunableNumber slowVelocity = new LoggedTunableNumber("Intake/SlowProfile/Velocity", IntakeConstants.SLOW_VELOCITY);
    // private LoggedTunableNumber slowAcceleration = new LoggedTunableNumber("Intake/SlowProfile/Acceleration", IntakeConstants.SLOW_ACCELERATION);

    // private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Intake/BrakeMotor", IntakeConstants.BRAKE_MOTOR);
    
    public Kicker(KickerIO io) {
        this.io = io;

        KickerConstants.LOGGED_KICKER_PERCENT.onChanged(Commands.runOnce(() -> testPercent = KickerConstants.LOGGED_KICKER_PERCENT.get()).ignoringDisable(true));

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
	
    public void setTestPercent() {
        setPercent(testPercent);
    }

    public Command setTestPercentCommand() {
        return Commands.runOnce(() -> setTestPercent());
    }
}
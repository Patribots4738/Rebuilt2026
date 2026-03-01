// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter.flywheel;

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


    @AutoLogOutput (key= "Subsystems/Shooter/Flywheel/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key= "Subsystems/Shooter/Flywheel/TargetLinearVelocity")
    private double targetVelocity = 0.0;
	
	private double percentOutput = 0.0;
    
	
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
							    .angularVelocity(RotationsPerSecond.of(inputs.velocityRadsPerSec)),
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
		Logger.processInputs("SubsystemInputs/Shooter/Flywheel", inputs);

        if (shouldRunSetpoint) {
            io.setPercent(percentOutput);
        } else {
            io.setNeutral();
        }

    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public void setVelocity(double velocity){
        targetVelocity = velocity / FlywheelConstants.CIRCUMFRENCE_METERS;
        shouldRunSetpoint = true;
    }

    public boolean atTargetVelocity(){
        return atVelocity(targetVelocity);
    }

    public boolean atVelocity(double velocity) {
        return MathUtil.isNear(getVelocity(), velocity, FlywheelConstants.VELOCITY_DEADBAND_METERS_PER_SEC);
    }

    public double getVelocity() {
        return inputs.velocityRadsPerSec *FlywheelConstants.CIRCUMFRENCE_METERS;
    }

    public double getTargetVelocity() {
        return targetVelocity * FlywheelConstants.CIRCUMFRENCE_METERS;
    }

    public Command setVelocityCommand(DoubleSupplier velocitySupplier) {
        return runOnce(() -> setVelocity(velocitySupplier.getAsDouble()));
    }

    public Command setVelocityCommand(double velocity) {
        return setVelocityCommand(() -> velocity);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command runCharacterization(double volts) {
        return runOnce(()-> io.runCharacterization(volts));
    }

    public double getVoltage() {
        return inputs.appliedOutputsVolts;
    } 
	

    
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return flywheelRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return flywheelRoutine.dynamic(direction);
	}
}
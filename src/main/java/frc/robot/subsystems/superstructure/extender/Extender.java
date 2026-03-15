// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.extender;



import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.Constants.LoggingConstants;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class Extender extends SubsystemBase {

    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    private double targetPosition = 0.0;
    @AutoLogOutput (key = "Subsystems/Extender/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;

    // private final SysIdRoutine extenderRoutine;
    private double testPosition;


    // private LoggedTunableNumber fastVelocity = new LoggedTunableNumber("Extender/FastProfile/Velocity", ExtenderConstants.FAST_VELOCITY);
    // private LoggedTunableNumber fastAcceleration = new LoggedTunableNumber("Extender/FastProfile/Acceleration", ExtenderConstants.FAST_ACCELERATION);
    // private LoggedTunableNumber slowVelocity = new LoggedTunableNumber("Extender/SlowProfile/Velocity", ExtenderConstants.SLOW_VELOCITY);
    // private LoggedTunableNumber slowAcceleration = new LoggedTunableNumber("Extender/SlowProfile/Acceleration", ExtenderConstants.SLOW_ACCELERATION);

    // private final LoggedTunableBoolean brakeMotor = new LoggedTunableBoolean("Extender/BrakeMotor", ExtenderConstants.BRAKE_MOTOR);
    
    public Extender(ExtenderIO io) {
        this.io = io;
        // this.extenderRoutine = new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //                 Volts.per(Seconds).of(1),
        //                 Volts.of(7),
        //                 Seconds.of(10),
        //                 (state) -> Logger.recordOutput("Subsystems/Extender/SysIdState", String.valueOf(state))
        //         ),
        //         new SysIdRoutine.Mechanism(
        //                 (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
        //                 log -> log.motor("extender")
        //                         .voltage(Volts.of(inputs.motorAppliedOutputVolts))
        //                         .linearPosition(Meters.of(inputs.motorPositionMeters))
        //                         .linearVelocity(MetersPerSecond.of(inputs.motorVelocityMetersPerSecond)),
        //                 this
        //         )
        // );
        
        // fastVelocity.onChanged().or(fastAcceleration.onChanged()).onTrue(Commands.runOnce(() -> fastProfile = new TrapezoidProfile(new Constraints(fastVelocity.get(), fastAcceleration.get()))).ignoringDisable(true));
        // slowVelocity.onChanged().or(slowAcceleration.onChanged()).onTrue(Commands.runOnce(() -> slowProfile = new TrapezoidProfile(new Constraints(slowVelocity.get(), slowAcceleration.get()))).ignoringDisable(true));

        ExtenderConstants.LOGGED_GAINS.onChanged(Commands.runOnce(() -> io.setGains(ExtenderConstants.LOGGED_GAINS.get())).ignoringDisable(true));
        ExtenderConstants.LOGGED_TEST_POSITION.onChanged(Commands.runOnce(() -> testPosition = Units.degreesToRadians(ExtenderConstants.LOGGED_TEST_POSITION.get())).ignoringDisable(true));
        ExtenderConstants.LOGGED_BRAKE_MODE.onChanged(Commands.runOnce(() -> io.setBrakeMode(ExtenderConstants.LOGGED_BRAKE_MODE.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Extender", inputs);

        Logger.recordOutput("Subsystems/Extender/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Extender/TargetPosition", targetPosition);

        if (shouldRunSetpoint) {
            io.setPosition(targetPosition);
        } else {
            io.setNeutral();
        }
        
        RobotContainer.components3d[LoggingConstants.EXTENDER_INDEX] = new Pose3d(LoggingConstants.EXTENDER_OFFSET,  new Rotation3d(0, -inputs.positionRads, 0) );
    }

    public void setPosition(double position) {
        position = MathUtil.clamp(position, ExtenderConstants.MIN_ANGLE_RADIANS, ExtenderConstants.MAX_ANGLE_RADIANS);
        targetPosition = position;
        shouldRunSetpoint = true;
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setTestPositionCommand() {
        return Commands.runOnce(() -> this.setTestPosition());
    }

    public void setTestPosition() {
        this.setPosition(testPosition);
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble()));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command resetEncodersCommand() {
        return runOnce(() -> io.resetEncoders(0));
    }

    public Command retractExtender() {
        return setPositionCommand(ExtenderConstants.STOW_POSITION_RADIANS);
    }

    public Command extendExtender() {
        return setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS);
    }
    
    public Command feedShooter() {
        return Commands.repeatingSequence(setPositionCommand(ExtenderConstants.HALF_STOW_RADIANS), Commands.waitSeconds(0.1), setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS), Commands.waitSeconds(0.1));
    }

    public Command feedShooterCommand() {
        return runOnce(() -> feedShooter());
    }

    public Command halfStowCommand() {
        return setPositionCommand(ExtenderConstants.HALF_STOW_RADIANS);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.positionRads, ExtenderConstants.DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.positionRads;
    }

    // public double getCharacterizationVelocity() {
    //     return inputs.motorVelocityMetersPerSecond / ExtenderConstants.VELOCITY_CONVERSION_FACTOR;
    // }

    // public double getVelocity() {
    //     return (inputs.motorVelocityMetersPerSecond);
    // }

    // public void runCharacterization(double input) {
    //     io.runCharacterization(input);
    // }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }
    
    // public Command quasistatic(SysIdRoutine.Direction direction) {
    //     return extenderRoutine.quasistatic(direction);
    // }
    
    // public Command dynamic(SysIdRoutine.Direction direction) {
    //     return extenderRoutine.dynamic(direction);
    // }
}

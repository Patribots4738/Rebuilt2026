// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.extender;


import frc.robot.RobotContainer;
import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.Constants.LoggingConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Extender extends SubsystemBase {

    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    private double targetPosition = 0.0;
    @AutoLogOutput (key = "Subsystems/Extender/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);

    // private final SysIdRoutine extenderRoutine;
    private double testPosition;
    private boolean wiggle;

    private TrapezoidProfile slowProfile = new TrapezoidProfile(new Constraints(ExtenderConstants.SLOW_VELOCITY, ExtenderConstants.SLOW_ACCELERATION));
    private TrapezoidProfile fastProfile = new TrapezoidProfile(new Constraints(ExtenderConstants.FAST_VELOCITY, ExtenderConstants.FAST_ACCELERATION));

    boolean shouldRunFast = true;


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

        // ExtenderConstants.LOGGED_GAINS.onChanged(Commands.runOnce(() -> io.setGains(ExtenderConstants.LOGGED_GAINS.get())).ignoringDisable(true));
        // ExtenderConstants.LOGGED_TEST_POSITION.onChanged(Commands.runOnce(() -> testPosition = Units.degreesToRadians(ExtenderConstants.LOGGED_TEST_POSITION.get())).ignoringDisable(true));
        // ExtenderConstants.LOGGED_BRAKE_MODE.onChanged(Commands.runOnce(() -> io.setBrakeMode(ExtenderConstants.LOGGED_BRAKE_MODE.get())).ignoringDisable(true));
        
        RobotContainer.desiredComponents3d[LoggingConstants.EXTENDER_INDEX] = new Pose3d(
                LoggingConstants.EXTENDER_OFFSET,
                new Rotation3d()
        );
        
        RobotContainer.desiredComponents3d[LoggingConstants.INTAKE_INDEX] = new Pose3d(
                LoggingConstants.INTAKE_OFFSET,
                new Rotation3d(0, LoggingConstants.INTAKE_ROTATION_OFFSET, 0)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Extender", inputs);

        Logger.recordOutput("Subsystems/Extender/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Extender/TargetPosition", targetPosition);

        if (shouldRunSetpoint) {
            // io.setPosition(targetPosition);

            setpoint = (shouldRunFast ? fastProfile : slowProfile).calculate(0.02, setpoint, targetState);
            io.setPosition(setpoint.position);
        } else {
            io.setNeutral();
        }
        
        RobotContainer.components3d[LoggingConstants.EXTENDER_INDEX] = new Pose3d(
                LoggingConstants.EXTENDER_OFFSET.getX() + Units.inchesToMeters(inputs.positionRads * 4.25),
                LoggingConstants.EXTENDER_OFFSET.getY(),
                LoggingConstants.EXTENDER_OFFSET.getZ(),
                new Rotation3d()
        );
        
        RobotContainer.components3d[LoggingConstants.INTAKE_INDEX] = new Pose3d(
                LoggingConstants.INTAKE_OFFSET,
                new Rotation3d(
                        0,
                        inputs.positionRads + LoggingConstants.INTAKE_ROTATION_OFFSET,
                        0
                )
        );
    }

    public void setPosition(double position, boolean runFast) {
        position = Math.clamp(position, ExtenderConstants.MIN_ANGLE_RADIANS, ExtenderConstants.MAX_ANGLE_RADIANS);
        // targetPosition = position;

        if (position != targetState.position) {
            setpoint.position = inputs.positionRads;
            setpoint.velocity = inputs.velocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true; 
        shouldRunFast = runFast;
        
        RobotContainer.desiredComponents3d[LoggingConstants.EXTENDER_INDEX] = new Pose3d(
                LoggingConstants.EXTENDER_OFFSET.getX() + Units.inchesToMeters(position * 4.25),
                LoggingConstants.EXTENDER_OFFSET.getY(),
                LoggingConstants.EXTENDER_OFFSET.getZ(),
                new Rotation3d()
        );
        
        RobotContainer.desiredComponents3d[LoggingConstants.INTAKE_INDEX] = new Pose3d(
                LoggingConstants.INTAKE_OFFSET,
                new Rotation3d(
                        0,
                        position + LoggingConstants.INTAKE_ROTATION_OFFSET,
                        0
                )
        );
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    // public Command setTestPositionCommand() {
    //     return Commands.runOnce(() -> this.setTestPosition());
    // }

    // public void setTestPosition() {
    //     this.setPosition(testPosition);
    // }

    public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier runFast) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble(), runFast.getAsBoolean()));
    }

    public Command setPositionCommand(double position, boolean runFast) {
        return setPositionCommand(() -> position, () -> runFast);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command resetEncodersCommand() {
        return runOnce(() -> io.resetEncoders(0));
    }

    public Command retractExtender() {
        return setPositionCommand(ExtenderConstants.STOW_POSITION_RADIANS, false);
    }

    public Command extendExtender() {
        return setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS, false);
    }
    
    public Command feedShooterCommand() {
        wiggle = true;
        return Commands.repeatingSequence(setPositionCommand(ExtenderConstants.HALF_STOW_RADIANS, true), Commands.waitSeconds(0.75), setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS, true), Commands.waitSeconds(1));
    }

    public Command feedShooterToggleCommand() {
        wiggle = !wiggle;
        return Commands.either(setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS, true), 
            feedShooterCommand(),
            () -> !wiggle);
    }

    public Command blipCommand() {
        return Commands.repeatingSequence(setPositionCommand(ExtenderConstants.BLIP_RADIANS, true), Commands.waitSeconds(0.25), setPositionCommand(ExtenderConstants.INTAKE_POSITION_RADIANS, true), Commands.waitSeconds(0.25));
    }

    public Command halfStowCommand() {
        return setPositionCommand(ExtenderConstants.HALF_STOW_RADIANS, false);
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

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }
    
    // public Command quasistatic(SysIdRoutine.Direction direction) {
    //     return extenderRoutine.quasistatic(direction);
    // }
    
    // public Command dynamic(SysIdRoutine.Direction direction) {
    //     return extenderRoutine.dynamic(direction);
    // }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climb;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.LoggingConstants;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    private boolean slam = false;
    @AutoLogOutput (key = "Subsystems/Climb/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);

        Logger.recordOutput("Subsystems/Climb/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Climb/TargetPosition", targetState.position);

        if (shouldRunSetpoint) {
            io.setPosition(setpoint.position, slam ? 1 : 0);
        } else {
            setpoint.position = inputs.positionMeters;
            setpoint.velocity = inputs.velocityMetersPerSec;
            io.setNeutral();
        }

        RobotContainer.components3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET.getX(), LoggingConstants.CLIMB_OFFSET.getY(), inputs.positionMeters,
            new Rotation3d()
        );
    }

    public void setPosition(double position, boolean slam) {
        position = MathUtil.clamp(position, 0, ClimbConstants.MAX_DISPLACEMENT_METERS);
        if (position != targetState.position) {
            setpoint.position = inputs.positionMeters;
            setpoint.velocity = inputs.velocityMetersPerSec;
        }
        targetState.position = position;
        this.slam = slam;
        shouldRunSetpoint = true;

        RobotContainer.components3d[LoggingConstants.CLIMB_INDEX] = new Pose3d(
            LoggingConstants.CLIMB_OFFSET.getX(), LoggingConstants.CLIMB_OFFSET.getY(), position,
            new Rotation3d()
        );
    }

    public void setNeutral() {
        io.setNeutral();
    }

    public void resetEncoder() {
        io.resetEncoder(0);
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier slamSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble(), slamSupplier.getAsBoolean())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position, boolean slam) {
        return setPositionCommand(() -> position, () -> slam);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command resetEncoderCommand() {
        return runOnce(this::resetEncoder);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.positionMeters, ClimbConstants.DEADBAND_METERS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetState.position);
    }

    public double getPosition() {
        return inputs.positionMeters;
    }

    public double getCharacterizationVelocity() {
        return inputs.velocityMetersPerSec / ClimbConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(0.5).per(Second),
                null, 
                null,
                (state) -> Logger.recordOutput("ClimbSysIdState", state.toString())
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

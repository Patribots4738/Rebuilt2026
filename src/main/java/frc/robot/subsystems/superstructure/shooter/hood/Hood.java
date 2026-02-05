package frc.robot.subsystems.superstructure.shooter.hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.LoggingConstants;

import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    private double yaw;
    @AutoLogOutput (key = "Subsystems/Shooter/Hood/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Shooter/Hood/ShouldRunFastProfile")
    private boolean shouldRunFast = true;
    
    private final SysIdRoutine hoodRoutine;

    private TrapezoidProfile fastProfile = new TrapezoidProfile(new Constraints(HoodConstants.FAST_VELOCITY, HoodConstants.FAST_ACCELERATION)); // use for tracking
    private TrapezoidProfile slowProfile = new TrapezoidProfile(new Constraints(HoodConstants.SLOW_VELOCITY, HoodConstants.SLOW_ACCELERATION)); // use to snap to the target

    public Hood(HoodIO io) { // the yaw should be the same as the turret's yaw
        this.io = io;
        this.yaw = yaw;
        this.hoodRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
                        log -> log.motor("hood")
                                .voltage(Volts.of(inputs.appliedOutputVolts))
                                .linearVelocity(MetersPerSecond.of(inputs.internalVelocityRadsPerSec)),
                        this
                )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Hood", inputs);

        Logger.recordOutput("Subsystems/Shooter/Hood/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Shooter/Hood/TargetPosition", targetState.position);

        if (shouldRunSetpoint) {
            setpoint = (shouldRunFast ? fastProfile : slowProfile).calculate(0.02, setpoint, targetState);
            // Logger.recordOutput("Subsystems/Wrist/Step/Velocity", setpoint.velocity);
            // Logger.recordOutput("Subsystems/Wrist/Step/Position", setpoint.position);
            io.setPosition(setpoint.position, 0);
        } else {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
            io.setNeutral();
        }

        RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            LoggingConstants.HOOD_OFFSET,
            new Rotation3d(0.0, 
                inputs.encoderAbsPositionRads, 
                RobotContainer.components3d[LoggingConstants.TURRET_INDEX].getRotation().getAngle())
        );
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position 
     * @param runFast
     */
    public void setPosition(double position, boolean runFast) {
        position = MathUtil.clamp(position, HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
        if (position != targetState.position) {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true;
        shouldRunFast = runFast;

        //TODO: create components3d for sim
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, BooleanSupplier runFastSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble(), runFastSupplier.getAsBoolean())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position, BooleanSupplier runFastSupplier) {
        return setPositionCommand(() -> position, runFastSupplier);
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.internalPositionRads, HoodConstants.POSITION_DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetState.position);
    }

    public double getPosition() {
        return inputs.internalPositionRads;
    }

    public double getVelocity() {
        return inputs.internalVelocityRadsPerSec;
    }

    public double getTargetPosition() {
        return targetState.position;
    }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }

    public double getCharacterizationVelocity() {
        return inputs.internalVelocityRadsPerSec / HoodConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }
    
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return hoodRoutine.quasistatic(direction);
    }
    
    public Command dynamic(SysIdRoutine.Direction direction) {
        return hoodRoutine.dynamic(direction);
    }
}

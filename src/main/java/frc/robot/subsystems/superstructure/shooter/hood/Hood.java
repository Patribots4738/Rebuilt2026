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
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.calc.ShotCalculations;

import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    @AutoLogOutput (key = "Subsystems/Shooter/Hood/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;

    private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(HoodConstants.VELOCITY, HoodConstants.ACCELERATION));
	
	private final SysIdRoutine hoodRoutine;
	
    public Hood(HoodIO io) {
        this.io = io;
        this.setDefaultCommand(this.setNeutralCommand());
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
                                .angularPosition(Radians.of(inputs.internalPositionRads))
                                .angularVelocity(RadiansPerSecond.of(inputs.internalVelocityRadsPerSec)),
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
            // TODO: consider if you need a profile since it continuously tracks unique state
            // it also resets this profile whenever the position changes, 
            // pretty much negating its use for target tracking
            // - you don't want predictability/smoothness, you want to get to setpoint ASAP
            setpoint = profile.calculate(0.02, setpoint, targetState);
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
     */
    public void setPosition(double position) {
        position = MathUtil.clamp(position, HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
        // TODO: This is where the profile is re-initialized
        // do you even need a profile for this/turret?
        if (position != targetState.position) {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true;
        //TODO: create components3d for sim
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
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

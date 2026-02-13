package frc.robot.subsystems.superstructure.shooter.turret;

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
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.calc.ShotCalculations;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    @AutoLogOutput (key = "Subsystems/Shooter/Turret/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    
    private final SysIdRoutine turretRoutine;

    private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(TurretConstants.VELOCITY, TurretConstants.ACCELERATION));

    public Turret(TurretIO io) {
        this.io = io;
        this.turretRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
                        log -> log.motor("turret")
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
        Logger.processInputs("SubsystemInputs/Shooter/Turret", inputs);

        Logger.recordOutput("Subsystems/Shooter/Turret/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Shooter/Turret/TargetPosition", targetState.position);

        if (shouldRunSetpoint) {
            setpoint = profile.calculate(0.02, setpoint, targetState);
            // Logger.recordOutput("Subsystems/Wrist/Step/Velocity", setpoint.velocity);
            // Logger.recordOutput("Subsystems/Wrist/Step/Position", setpoint.position);
            io.setPosition(setpoint.position, 0);
        } else {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
            io.setNeutral();
        }
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position in radians
     */
    public void setPosition(double position) {
        position = position % (TurretConstants.MAX_ANGLE_RADIANS - TurretConstants.MIN_ANGLE_RADIANS);
        if (position != targetState.position) {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true;

        RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            LoggingConstants.TURRET_OFFSET,
            new Rotation3d(0, 0, inputs.encoderAbsPositionRads)
        );
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
        return MathUtil.isNear(position, inputs.internalPositionRads, TurretConstants.ROTATION_TOLERANCE_RADIANS);
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
        return inputs.internalVelocityRadsPerSec / TurretConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }
    
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return turretRoutine.quasistatic(direction);
    }
    
    public Command dynamic(SysIdRoutine.Direction direction) {
        return turretRoutine.dynamic(direction);
    }
}

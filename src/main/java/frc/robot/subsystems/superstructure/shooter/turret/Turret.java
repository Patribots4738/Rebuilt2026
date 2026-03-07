package frc.robot.subsystems.superstructure.shooter.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import frc.robot.util.Constants.TurretConstants;
import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Rotation2d targetPosition;
    private Rotation2d currentPosition;
    @AutoLogOutput (key = "Subsystems/Shooter/Turret/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    
    private double testPositionDegrees;

    private final SysIdRoutine turretRoutine;


    public Turret(TurretIO io) {
        this.io = io;

        TurretConstants.LOGGED_TURRET_GAINS.onChanged(Commands.runOnce(() -> io.setGains(TurretConstants.LOGGED_TURRET_GAINS.get())).ignoringDisable(true));
        TurretConstants.LOGGED_TEST_POSITION_DEGREES.onChanged(Commands.runOnce(() -> testPositionDegrees = TurretConstants.LOGGED_TEST_POSITION_DEGREES.get()).ignoringDisable(true));

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
        Logger.recordOutput("Subsystems/Shooter/Turret/TargetPositionDegrees", targetPosition.getDegrees());
        Logger.recordOutput("Subsystems/Shooter/Turret/CurrentPositionDegrees", );
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position in radians 
     * 
     * TODO: make position more consistent by
     * - making it a Rotation2d (easier to work with)
     * - @see MathUtil.angleModulus (or inputModulus(position, 0, 2pi)) rather than normal '%' (negative sign safe) (-1 % 360 = -1)
     * - check if value is outside of MIN and MAX constants range (inside of hardstop)
     * - Immediately either: 
     *      return this function and not set anything, keeping the turret temporarily stationary until valid values are found (oliver and alexander preference)
     * or
     *      Get the closest possible value to the deadzone, meaning it would hug the deadzone wall and possibly oscillate
     */
    public void setPosition(Rotation2d position) {
        if (position.getRadians() < TurretConstants.MAX_ANGLE_RADIANS && position.getRadians() > TurretConstants.MIN_ANGLE_RADIANS) {
            targetPosition = position;
        }
        else {
            targetPosition = inputs.encoderAbsPositionRads * TurretConst
        }
        if (position != targetState.position) {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true;

        RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            LoggingConstants.TURRET_OFFSET,
            new Rotation3d(0, 0, targetPosition.getRadians())
        );
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble())).andThen(Commands.waitUntil(this::atTargetPosition));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }

    public void setTestPositionDegrees() {
        setPosition(Units.degreesToRadians(testPositionDegrees));
    }

    public Command setTestPositionCommand() {
        return Commands.runOnce(() -> setTestPositionDegrees());
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

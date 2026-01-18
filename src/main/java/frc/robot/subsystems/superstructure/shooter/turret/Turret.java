package frc.robot.subsystems.superstructure.shooter.turret;

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
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private State targetState = new State(0, 0);
    private State setpoint = new State(0, 0);
    @AutoLogOutput (key = "Subsystems/Shooter/Turret/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Shooter/Turret/ShouldRunFastProfile")
    private boolean shouldRunFast = true;

    private TrapezoidProfile fastProfile = new TrapezoidProfile(new Constraints(TurretConstants.FAST_VELOCITY, TurretConstants.FAST_ACCELERATION)); // use for tracking
    private TrapezoidProfile slowProfile = new TrapezoidProfile(new Constraints(TurretConstants.SLOW_VELOCITY, TurretConstants.SLOW_ACCELERATION)); // use to snap to the target


    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Turret", inputs);

        Logger.recordOutput("Subsystems/Shooter/Turret/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Shooter/Turret/TargetPosition", targetState.position);
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position 
     * @param runFast
     */
    public void setPosition(double position, boolean runFast) {
        position = MathUtil.clamp(position, TurretConstants.MIN_ANGLE_RADIANS, TurretConstants.MAX_ANGLE_RADIANS);
        if (position != targetState.position) {
            setpoint.position = inputs.internalPositionRads;
            setpoint.velocity = inputs.internalVelocityRadsPerSec;
        }
        targetState.position = position;
        shouldRunSetpoint = true;
        shouldRunFast = runFast;

        RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            RobotContainer.components3d[LoggingConstants.HOOD_INDEX].getX(), 
            RobotContainer.components3d[LoggingConstants.HOOD_INDEX].getY(),
            RobotContainer.components3d[LoggingConstants.HOOD_INDEX].getZ(),
            new Rotation3d(0, 0, inputs.encoderAbsPositionRads)
        );
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
        return MathUtil.isNear(position, inputs.internalPositionRads, TurretConstants.POSITION_DEADBAND_RADIANS);
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
        return inputs.internalVelocityRadsPerSec / TurretConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    // TODO: make this work :D
    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(0.5).per(Second),
                null, 
                Seconds.of(2),
                (state) -> Logger.recordOutput("TurretSysIdState", state.toString())
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

package frc.robot.subsystems.superstructure.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.calc.ShotCalculations;

public class Flywheel extends SubsystemBase {
    
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final Swerve swerveInstance;
    private final int passIndex = 0;

    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/TargetVelocity")
    private double targetVelocity = 0.0;
    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/TrackTarget")
    private boolean trackTarget = false;

    public Flywheel(FlywheelIO io, Swerve swerve) {
        this.io = io;
        swerveInstance = swerve;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Flywheel", inputs);

        if (trackTarget) {
            setVelocity(ShotCalculations.getParameters(swerveInstance).flywheelSpeed());
        }

        // Run setpoint on RIO to minimize CAN utilization
        if (shouldRunSetpoint) {
            io.setVelocity(targetVelocity);
        } else {
            io.setNeutral();
        }
    }

    
    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setTrackTargetCommand(boolean track) {
        return runOnce(() -> setTrackTarget(track));
    }

    public void setTrackTarget(boolean track) {
        trackTarget = track;
    } 

    /**
     * Sets the velocity of the wheel using the desired tangential velocity
     * 
     * @param velocity tangential velocity
     */
    public void setVelocity(double velocity) {
        targetVelocity = velocity / FlywheelConstants.CIRCUMFERENCE_METERS;
        shouldRunSetpoint = true;
    }

    public boolean atTargetVelocity() {
        return atVelocity(targetVelocity);
    }

    public boolean atVelocity(double velocity) {
        return MathUtil.isNear(getVelocity(), velocity, FlywheelConstants.VELOCITY_DEADBAND_METERS_PER_SEC);
    }

    /**
     * Returns the tangential velocity of the flywheel
     * 
     * @return the tangential velocity of the flywheel
     */
    public double getVelocity() {
        return inputs.velocityRadsPerSec * FlywheelConstants.CIRCUMFERENCE_METERS;
    }

    public double getTargetVelocity() {
        return targetVelocity * FlywheelConstants.CIRCUMFERENCE_METERS;
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

    public boolean isTracking() {
        return trackTarget;
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                // Gaslight SysId since motor is actually running amps instead of volts, feedforwards should still be accurate
                Volts.of(0.5).per(Second),
                null, 
                Seconds.of(2),
                (state) -> Logger.recordOutput("FlywheelSysIdState", state.toString())
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

package frc.robot.subsystems.superstructure.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
    
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private boolean shouldRunSetpoint = false;
    private double targetVelocity = 0.0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Flywheel", inputs);

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

    /**
     * Sets the tangential velocity of the wheel
     * 
     * @param velocity tangential velocity
     */
    public void setVelocity(double velocity) {
        velocity /= FlywheelConstants.CIRCUMFERENCE_METERS;
        shouldRunSetpoint = true;
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
}

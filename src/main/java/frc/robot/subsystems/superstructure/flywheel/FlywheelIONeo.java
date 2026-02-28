package frc.robot.subsystems.superstructure.flywheel;

import com.revrobotics.REVLibError;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class FlywheelIONeo implements FlywheelIO {

    private final Neo motor;
    
    public FlywheelIONeo() {
        motor = new Neo(FlywheelConstants.CAN_ID, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setInverted(FlywheelConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit((int) FlywheelConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        configMotor(motor);
        setBrakeMode(FlywheelConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.percentOutput = motor.getAppliedOutput();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
        inputs.positionRotations = motor.getPosition();
        inputs.velocityRotationsPerSec = motor.getVelocity() / 60;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            motor.setBrakeMode();

        } else {
            motor.setCoastMode();

        }
    }
    
    @Override
    public void setPercent(double percent) {
        motor.setTargetPercent(percent);
    }
}
package frc.robot.subsystems.superstructure.kicker;

import com.revrobotics.REVLibError;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class KickerIONeo implements KickerIO {

    private final Neo motor;
    
    public KickerIONeo() {
        motor = new Neo(KickerConstants.CAN_ID, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setInverted(KickerConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit((int) KickerConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        configMotor(motor);
        setBrakeMode(KickerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorPositionMeters = motor.getPosition();
        inputs.motorTargetPositionMeters = motor.getTargetPosition();
        inputs.motorVelocityMetersPerSecond = motor.getVelocity();
        inputs.motorAppliedOutputVolts = motor.getBusVoltage();
        inputs.motorTorqueCurrentAmps = motor.getOutputCurrent();
        inputs.motorTemperatureCelsius = motor.getMotorTemperature();
        inputs.positionRads = motor.getPosition() / KickerConstants.POSITION_CONVERSION_FACTOR * (2.0 * Math.PI);
    }

    @Override
    public void setPosition(double position) {
        motor.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        motor.setVoltage(input);
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
    public void resetEncoders(double position) {
        motor.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setPID(constants);
    }
}
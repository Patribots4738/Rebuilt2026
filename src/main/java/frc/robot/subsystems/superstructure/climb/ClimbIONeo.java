package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class ClimbIONeo implements ClimbIO {
     private final Neo motor;

    public ClimbIONeo() {
        motor = new Neo(ClimbConstants.CAN_ID, false);
        configMotor();
    }

    private void configMotor() {
        resetEncoder(0);
        motor.setPID(ClimbConstants.GAINS);
        motor.setInverted(ClimbConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);
        motor.setBrakeMode();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.positionMeters = motor.getPosition();
        inputs.targetPositionMeters = motor.getTargetPosition();
        inputs.velocityMetersPerSec = motor.getVelocity();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
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
    public void resetEncoder(double position) {
        motor.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setPID(constants);
    }
}

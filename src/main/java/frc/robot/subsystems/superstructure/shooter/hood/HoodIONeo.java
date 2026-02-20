package frc.robot.subsystems.superstructure.shooter.hood;

import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class HoodIONeo implements HoodIO {
    private final Neo motor;

    public HoodIONeo() {
        motor = new Neo(HoodConstants.CAN_ID, false, false, true);
        configMotor();
    }

    private void configMotor() {
        motor.setPID(HoodConstants.GAINS);
        motor.setInverted(HoodConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(HoodConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(HoodConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) HoodConstants.CURRENT_LIMIT);
        setBrakeMode(HoodConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.internalPositionRads = motor.getPosition();
        inputs.internalVelocityRadsPerSec = motor.getVelocity();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
        inputs.encoderAbsPositionRads = inputs.internalPositionRads;
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
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
    public void setGains(GainConstants constants) {
        motor.setPID(constants);
    }

    @Override
    public void resetEncoders(double position) {
        motor.resetEncoder(position);

    }
}

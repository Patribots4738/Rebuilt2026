package frc.robot.subsystems.superstructure.extender;

import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class ExtenderIONeo implements ExtenderIO {

    private final Neo motor;


    public ExtenderIONeo() {
        motor = new Neo(ExtenderConstants.CAN_ID, false);

        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPID(ExtenderConstants.GAINS);
        motor.setInverted(ExtenderConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(ExtenderConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ExtenderConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) ExtenderConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        configMotor(motor);

        setBrakeMode(ExtenderConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {

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
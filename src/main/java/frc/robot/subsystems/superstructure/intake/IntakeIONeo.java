package frc.robot.subsystems.superstructure.intake;

import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class IntakeIONeo implements IntakeIO {

    private final Neo motor;


    public IntakeIONeo() {
        motor = new Neo(IntakeConstants.MOTOR_CAN_ID, false);

        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPID(IntakeConstants.GAINS);
        motor.setInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        configMotor(motor);

        setBrakeMode(IntakeConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

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
package frc.robot.subsystems.superstructure.spindexer;

import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class SpindexerIONeo implements SpindexerIO {

    private final Neo motor;


    public SpindexerIONeo() {
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
    public void updateInputs(SpindexerIOInputs inputs) {

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
    public void setPercent(double percent){
        motor.setTargetPercent(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
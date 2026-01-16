package frc.robot.subsystems.superstructure.intake;

import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class IntakeIONeo implements IntakeIO {

    private final Neo leader;
    private final Neo follower;

    public IntakeIONeo() {
        leader = new Neo(IntakeConstants.LEADER_CAN_ID, false);
        follower = new Neo(IntakeConstants.FOLLOWER_CAN_ID, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPID(IntakeConstants.GAINS);
        motor.setOutputInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(IntakeConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setVoltage(input);
        follower.setVoltage(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            leader.setBrakeMode();
            follower.setBrakeMode();
        } else {
            leader.setCoastMode();
            follower.setCoastMode();
        }
    }

    @Override
    public void resetEncoders(double position) {
        leader.resetEncoder(position);
        follower.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        leader.setPID(constants);
        follower.setPID(constants);
    }

    
}
package frc.robot.subsystems.superstructure.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.OIConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class IntakeIOKraken implements IntakeIO {

    private final Kraken leader;
    private final Kraken follower;

    public IntakeIOKraken() {
        leader = new Kraken(IntakeConstants.LEADER_CAN_ID, true, true, ControlPreference.TORQUE_CURRENT);
        follower = new Kraken(IntakeConstants.FOLLOWER_CAN_ID, true, true, ControlPreference.TORQUE_CURRENT);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(IntakeConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(0.0, IntakeConstants.MAX_DISPLACEMENT_METERS);
        motor.setGains(IntakeConstants.GAINS);
        motor.setStatorCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-IntakeConstants.CURRENT_LIMIT, IntakeConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        leader.setMotorInverted(IntakeConstants.MOTOR_INVERTED);
        follower.setMotorInverted(!IntakeConstants.MOTOR_INVERTED);
        configMotor(leader);
        configMotor(follower);
        setBrakeMode(IntakeConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
  
    }

    @Override
    public void setNeutral() {
        leader.setNeutral();
        follower.setNeutral();
    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setTorqueCurrentOutput(input);
        follower.setTorqueCurrentOutput(input);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        follower.setBrakeMode(brake);
    }

    @Override
    public void resetEncoders(double position) {
        leader.resetEncoder(position);
        follower.resetEncoder(position);
    }
    
    @Override
    public void setGains(GainConstants constants) {
        leader.setGains(constants);
        follower.setGains(constants);
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        leader.configureMotionMagic(velocity, acceleration, jerk);
        follower.configureMotionMagic(velocity, acceleration, jerk);
    }

}
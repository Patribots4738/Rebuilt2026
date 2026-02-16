package frc.robot.subsystems.superstructure.extender;

import frc.robot.util.Constants.ExtenderConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class ExtenderIOKraken implements ExtenderIO {

    private final Kraken motor;

    public ExtenderIOKraken() {
        motor = new Kraken(ExtenderConstants.CAN_ID, ExtenderConstants.MOTOR_TYPE, true, true, ControlPreference.TORQUE_CURRENT);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(ExtenderConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(ExtenderConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(ExtenderConstants.MIN_ANGLE_RADIANS, ExtenderConstants.MAX_ANGLE_RADIANS);
        motor.setGains(ExtenderConstants.GAINS);
        motor.setStatorCurrentLimit(ExtenderConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-ExtenderConstants.CURRENT_LIMIT, ExtenderConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        motor.setMotorInverted(ExtenderConstants.MOTOR_INVERTED);

        configMotor(motor);
        setBrakeMode(ExtenderConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
  
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();

    }

    @Override
    public void setPosition(double position) {
        motor.setTargetPosition(position);

    }

    @Override
    public void runCharacterization(double input) {
        motor.setTorqueCurrentOutput(input);

    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);

    }

    @Override
    public void resetEncoders(double position) {
        motor.resetEncoder(position);

    }
    
    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants);

    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        motor.configureMotionMagic(velocity, acceleration, jerk);

    }

}
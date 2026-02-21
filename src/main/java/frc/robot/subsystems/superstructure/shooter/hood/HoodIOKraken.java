package frc.robot.subsystems.superstructure.shooter.hood;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class HoodIOKraken implements HoodIO {
    private final Kraken motor;

    public HoodIOKraken() {
        motor = new Kraken(HoodConstants.CAN_ID, HoodConstants.MOTOR_TYPE, true, true, ControlPreference.VOLTAGE);
        configMotor();
    }

    public void configMotor() {
        motor.setBrakeMode(HoodConstants.BRAKE_MOTOR);
        motor.setUnitConversionFactor(HoodConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(HoodConstants.MOTOR_INVERTED);
        motor.setGearRatio(HoodConstants.GEAR_RATIO);
        motor.setGains(HoodConstants.GAINS);
        motor.setStatorCurrentLimit(HoodConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-HoodConstants.CURRENT_LIMIT, HoodConstants.CURRENT_LIMIT);
        motor.setBrakeMode(HoodConstants.BRAKE_MOTOR);
        motor.resetEncoder(0.0);
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void resetEncoders(double position) {
        motor.resetEncoder(position);

    }

    @Override
    public void runCharacterization(double input) {}

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants.withG(0.0));
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        motor.configureMotionMagic(velocity, acceleration, jerk);
    }
}

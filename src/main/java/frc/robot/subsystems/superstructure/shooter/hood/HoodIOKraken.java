package frc.robot.subsystems.superstructure.shooter.hood;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class HoodIOKraken implements HoodIO {
    private final Kraken motor;
    private final CANCoderCustom encoder;
    private boolean encoderUpdated = false;

    public HoodIOKraken() {
        motor = new Kraken(TurretConstants.MOTOR_CAN_ID, TurretConstants.MOTOR_TYPE, CANConstants.SUPERSTRUCTURE_BUS, true, true, ControlPreference.TORQUE_CURRENT);
        encoder = new CANCoderCustom(TurretConstants.ENCODER_CAN_ID, CANConstants.SUPERSTRUCTURE_BUS);
        configMotor();
        configEncoder();
    }

    public void configMotor() {
        motor.setUnitConversionFactor(HoodConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(HoodConstants.MOTOR_INVERTED);
        if (FieldConstants.IS_REAL) {
            motor.setEncoder(encoder.getDeviceID(), MK5nSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        }
        motor.setGains(HoodConstants.GAINS);
        motor.setStatorCurrentLimit(HoodConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-HoodConstants.CURRENT_LIMIT, HoodConstants.CURRENT_LIMIT);
        motor.setBrakeMode(HoodConstants.BRAKE_MOTOR);
        motor.setTelemetryPreference(TelemetryPreference.DEFAULT);
    }

    public void configEncoder() {
        encoder.configureMagnetSensor(false, HoodConstants.ABSOLUTE_ENCODER_OFFSET);
        encoder.setPositionConversionFactor(HoodConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(HoodConstants.VELOCITY_CONVERSION_FACTOR);
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

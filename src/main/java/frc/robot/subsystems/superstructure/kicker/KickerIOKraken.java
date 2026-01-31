package frc.robot.subsystems.superstructure.kicker;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class KickerIOKraken implements KickerIO {

    private final Kraken motor;

    public KickerIOKraken() {
        motor = new Kraken(KickerConstants.CAN_ID, KickerConstants.MOTOR_TYPE, CANConstants.SUPERSTRUCTURE_BUS,true, true, ControlPreference.TORQUE_CURRENT);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(KickerConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(KickerConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(0.0, KickerConstants.MAX_DISPLACEMENT_METERS);
        motor.setGains(KickerConstants.GAINS);
        motor.setStatorCurrentLimit(KickerConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-KickerConstants.CURRENT_LIMIT, KickerConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        resetEncoders(0);
        motor.setMotorInverted(KickerConstants.MOTOR_INVERTED);

        configMotor(motor);
        setBrakeMode(KickerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorConnected = motor.isConnected();
        inputs.motorPositionMeters = motor.getPosition().getValueAsDouble();
        inputs.motorTargetPositionMeters = motor.getTargetPosition();
        inputs.motorVelocityMetersPerSecond = motor.getVelocity().getValueAsDouble();
        inputs.motorAppliedOutputVolts = motor.getVoltageAsDouble();
        inputs.motorTorqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.motorTemperatureCelsius = motor.getTemperatureAsDouble();
        inputs.positionRads = motor.getPosition().getValueAsDouble() / KickerConstants.POSITION_CONVERSION_FACTOR * (2.0 * Math.PI);
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
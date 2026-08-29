package frc.robot.subsystems.superstructure.shooter.hood;

import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class HoodIOKraken implements HoodIO {
    private final Kraken motor;
    public HoodIOKraken() {
        motor = new Kraken(HoodConstants.CAN_ID, HoodConstants.MOTOR_TYPE, true, false, ControlPreference.TORQUE_CURRENT);
        configMotor();
    }

    public void configMotor() {
        motor.setGearRatio(HoodConstants.GEAR_RATIO);
        motor.setUnitConversionFactor(HoodConstants.POSITION_CONVERSION_FACTOR);
        motor.setMotorInverted(HoodConstants.MOTOR_INVERTED);
        motor.setGains(HoodConstants.GAINS, 0);
        motor.setStatorCurrentLimit(HoodConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-HoodConstants.CURRENT_LIMIT, HoodConstants.CURRENT_LIMIT);
        motor.setBrakeMode(HoodConstants.BRAKE_MOTOR);
        motor.resetEncoder(0.0);
        motor.setSoftLimits(HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.internalPositionRads = motor.getPositionAsDouble();
        inputs.internalVelocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
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

    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    public void setCurrent(double amps) {
        motor.setTorqueCurrentOutput(amps);
    }
}

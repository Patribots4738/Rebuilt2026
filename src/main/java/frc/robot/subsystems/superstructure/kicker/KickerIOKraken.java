package frc.robot.subsystems.superstructure.kicker;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.CANRangeConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.CANRangeCustom;

public class KickerIOKraken implements KickerIO {

    private final Kraken motor;
    private final CANRangeCustom range;

    public KickerIOKraken() {
        motor = new Kraken(KickerConstants.CAN_ID, KickerConstants.MOTOR_TYPE, true, true, ControlPreference.TORQUE_CURRENT);
        range = new CANRangeCustom(CANRangeConstants.CANRANGE_ID);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(KickerConstants.MOTOR_INVERTED);
        motor.setGearRatio(KickerConstants.GEAR_RATIO);
        motor.setSupplyCurrentLimit(KickerConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(KickerConstants.CURRENT_LIMIT);
        setBrakeMode(KickerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();

        inputs.isRangeTripped = range.getIsDetectedAsBoolean();
        inputs.rangeDistance = range.getDistanceAsDouble();
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setPercent(double percent) {
        motor.setPercentOutput(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltageOutput(volts);
    }

    @Override
    public void setCurrent(double amps) {
        motor.setTorqueCurrentOutput(amps);
    }
}
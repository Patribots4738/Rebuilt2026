package frc.robot.subsystems.superstructure.flywheel;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.CANRangeConstants;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.CANRangeCustom;

public class FlywheelIOKracken implements FlywheelIO {

    private final Kraken motor;
    private final CANRangeCustom range;

    public FlywheelIOKracken() {
        motor = new Kraken(FlywheelConstants.CAN_ID, FlywheelConstants.MOTOR_TYPE, CANConstants.RIO_BUS,true, true, ControlPreference.TORQUE_CURRENT);
        range = new CANRangeCustom(CANRangeConstants.CANRANGE_ID);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(FlywheelConstants.MOTOR_INVERTED);
        motor.setGearRatio(FlywheelConstants.GEAR_RATIO);
        motor.setSupplyCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        setBrakeMode(FlywheelConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();
        inputs.positionRotations = motor.getPositionAsDouble();
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
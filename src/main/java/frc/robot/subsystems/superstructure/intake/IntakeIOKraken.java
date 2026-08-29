package frc.robot.subsystems.superstructure.intake;

import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class IntakeIOKraken implements IntakeIO {

    private final Kraken motor;

    public IntakeIOKraken() {
        motor = new Kraken(IntakeConstants.CAN_ID, IntakeConstants.MOTOR_TYPE, true, true, ControlPreference.TORQUE_CURRENT);
        configMotors();
    }

    private void configMotor(Kraken motor) {
        motor.setGearRatio(IntakeConstants.GEAR_RATIO);
        motor.setSupplyCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        setBrakeMode(IntakeConstants.BRAKE_MOTOR);
    }

    private void configMotors() {
        motor.setMotorInverted(IntakeConstants.MOTOR_INVERTED);
        configMotor(motor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();
        inputs.positionRotations = motor.getPositionAsDouble();
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();

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

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);

    }
}

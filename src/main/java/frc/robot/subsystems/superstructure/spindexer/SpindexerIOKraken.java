package frc.robot.subsystems.superstructure.spindexer;

import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class SpindexerIOKraken implements SpindexerIO {
    
    private final Kraken motor;
    
        public SpindexerIOKraken() {
        motor = new Kraken(SpindexerConstants.CAN_ID, SpindexerConstants.MOTOR_TYPE, true, true, ControlPreference.VOLTAGE);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(SpindexerConstants.MOTOR_INVERTED);
        motor.setSupplyCurrentLimit(SpindexerConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(SpindexerConstants.CURRENT_LIMIT);
        setBrakeMode(SpindexerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();
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

package frc.robot.subsystems.superstructure.shooter.flywheel;

import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class FlywheelIOKraken implements FlywheelIO {
    
    private final Kraken motor;

    public FlywheelIOKraken() {
        motor = new Kraken(FlywheelConstants.CAN_ID, FlywheelConstants.MOTOR_TYPE, true, true, ControlPreference.VOLTAGE);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(FlywheelConstants.MOTOR_INVERTED);
        motor.setSupplyCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        motor.setGains(FlywheelConstants.GAINS);
        setBrakeMode(FlywheelConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetVelocityRadsPerSec = motor.getTargetVelocity();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRadsPerSec = motor.getVelocityAsDouble();
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
    public void setVelocity(double velocity) {
        motor.setTargetVelocity(velocity);
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
    public void setGains(GainConstants constants) {
        motor.setGains(constants.withG(0.0));
    }
}

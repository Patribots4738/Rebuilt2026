package frc.robot.subsystems.superstructure.kicker;

import com.revrobotics.REVLibError;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class KickerIONeo implements KickerIO {

    private final Neo motor;
    
    public KickerIONeo() {
        motor = new Neo(KickerConstants.CAN_ID, false);
        configMotors();
    }

    private void configMotor(Neo motor) {
        motor.setPID(KickerConstants.GAINS);
        motor.setInverted(KickerConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(KickerConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(KickerConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) KickerConstants.CURRENT_LIMIT);
    }

    private void configMotors() {
        configMotor(motor);
        setBrakeMode(KickerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.percentOutput = motor.getAppliedOutput();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            motor.setBrakeMode();

        } else {
            motor.setCoastMode();

        }
    }
    
    @Override
    public void setPercent(double percent) {
        motor.setTargetPercent(percent);
    }
}
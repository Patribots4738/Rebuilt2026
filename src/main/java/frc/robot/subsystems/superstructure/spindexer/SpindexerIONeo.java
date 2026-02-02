package frc.robot.subsystems.superstructure.spindexer;


import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.hardware.rev.Neo;

public class SpindexerIONeo implements SpindexerIO {

    private final Neo motor;


    public SpindexerIONeo() {
        motor = new Neo(SpindexerConstants.CAN_ID, false);

        configMotors();
    }

    private void configMotors() {
        resetEncoders(0);
        setBrakeMode(SpindexerConstants.BRAKE_MOTOR);
        motor.setInverted(SpindexerConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit((int) SpindexerConstants.CURRENT_LIMIT);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void runCharacterization(double input) {
        motor.setVoltage(input);

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
    public void resetEncoders(double position) {
        motor.resetEncoder(position);
        
    }

    @Override
    public void setPercent(double percent){
        motor.setTargetPercent(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
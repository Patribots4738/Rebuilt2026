package frc.robot.subsystems.superstructure.spindexer;


import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.hardware.rev.Neo;

public class SpindexerIONeo implements SpindexerIO {

    private final Neo motor;


    public SpindexerIONeo() {
        motor = new Neo(SpindexerConstants.CAN_ID, false);

        configMotor();
    }

    private void configMotor() {
        resetEncoders(0);
        setBrakeMode(SpindexerConstants.BRAKE_MOTOR);
        motor.setInverted(SpindexerConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit((int) SpindexerConstants.CURRENT_LIMIT);
        motor.setVelocityConversionFactor(SpindexerConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.internalVelocityRadsPerSec = motor.getVelocityRadsPerSec();
        inputs.appliedOutputVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
        inputs.percentOutput = motor.getAppliedOutput();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.velocityRotationsPerSec = motor.getVelocity() / 60.0;
        inputs.positionRotations = motor.getPosition();
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
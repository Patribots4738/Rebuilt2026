package frc.robot.subsystems.superstructure.shooter.turret;

import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class TurretIONeo implements TurretIO {
    private final Neo motor;

    public TurretIONeo() {
        motor = new Neo(TurretConstants.MOTOR_CAN_ID, false, false, true);
        configMotor();
    }

    private void configMotor() {
        motor.setPID(TurretConstants.GAINS);
        motor.setInverted(TurretConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(TurretConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) TurretConstants.CURRENT_LIMIT);
        setBrakeMode(TurretConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.internalPositionRads = motor.getPositionRads();
        inputs.internalVelocityRadsPerSec = motor.getVelocityRadsPerSec();
        inputs.targetPositionRads = motor.getTargetPositionRads();
        inputs.appliedOutputVolts = motor.getBusVoltage();
        inputs.torqueCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
        inputs.encoderAbsPositionRads = inputs.internalPositionRads;
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
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
    public void setGains(GainConstants constants) {
        motor.setPID(constants);
    }
}

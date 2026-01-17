package frc.robot.subsystems.superstructure.shooter.turret;

import frc.robot.subsystems.superstructure.shooter.turret.TurretIO.TurretIOInputs;
import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class TurretIOKraken implements TurretIO {
    private final Kraken motor;
    private final CANCoderCustom encoder;
    private boolean encoderUpdated = false;

    public TurretIOKraken() {
        motor = new Kraken(TurretConstants.MOTOR_CAN_ID, TurretConstants.MOTOR_TYPE, CANConstants.SUPERSTRUCTURE_BUS, true, true, ControlPreference.TORQUE_CURRENT);
        encoder = new CANCoderCustom(TurretConstants.ENCODER_CAN_ID, CANConstants.SUPERSTRUCTURE_BUS);
        configMotor();
        configEncoder();
    }

    private void configMotor() {
        motor.setUnitConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        motor.setSoftLimits(TurretConstants.MIN_ANGLE_RADIANS, TurretConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(TurretConstants.MOTOR_INVERTED);
        if (FieldConstants.IS_REAL) {
            motor.setEncoder(encoder.getDeviceID(), MK5nSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        }
        motor.setGains(TurretConstants.GAINS);
        motor.setStatorCurrentLimit(TurretConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-TurretConstants.CURRENT_LIMIT, TurretConstants.CURRENT_LIMIT);
        motor.setBrakeMode(TurretConstants.BRAKE_MOTOR);
        motor.setTelemetryPreference(TelemetryPreference.DEFAULT);
    }

    private void configEncoder() {
        encoder.configureMagnetSensor(false, TurretConstants.ABSOLUTE_ENCODER_OFFSET);
        encoder.setPositionConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(TurretConstants.VELOCITY_CONVERSION_FACTOR);
    }

    /**
     * Updates the inputs sent to the Mk4c module.
     */
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Call refreshALl() to refresh all status signals, and check in on him :)
        inputs.motorConnected = motor.refreshSignals().isOK();
        if (inputs.encoderConnected) {
            inputs.encoderAbsPositionRads = encoder.getPositionAsDouble();
            if (!encoderUpdated) {
                motor.resetEncoder(inputs.encoderAbsPositionRads > TurretConstants.MAX_ANGLE_RADIANS ? inputs.encoderAbsPositionRads - 2 * Math.PI : inputs.encoderAbsPositionRads);
                encoderUpdated = true;
            }
        } else {
            inputs.encoderAbsPositionRads = inputs.internalPositionRads;
        }   
        inputs.internalPositionRads = motor.getPositionAsDouble();
        inputs.internalVelocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setPosition(double position, double feedForward) {}

    @Override
    public void setNeutral() {}

    @Override
    public void runCharacterization(double input) {}

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void resetEncoders(double position) {
        
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants.withG(0.0));
    }

    @Override
    public void configureProfile(double velocity, double acceleration, double jerk) {
        motor.configureMotionMagic(velocity, acceleration, jerk);
    }

}

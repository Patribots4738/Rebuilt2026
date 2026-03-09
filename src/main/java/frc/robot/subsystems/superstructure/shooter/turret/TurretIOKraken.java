package frc.robot.subsystems.superstructure.shooter.turret;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.FieldConstants;
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
        motor = new Kraken(TurretConstants.MOTOR_CAN_ID, TurretConstants.MOTOR_TYPE, true, true, ControlPreference.TORQUE_CURRENT);
        encoder = new CANCoderCustom(TurretConstants.ENCODER_CAN_ID);
        configEncoder();
        configMotor();
    }

    private void configMotor() {
        motor.setUnitConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        // motor.setSoftLimits(TurretConstants.MIN_ANGLE_RADIANS, TurretConstants.MAX_ANGLE_RADIANS);
        motor.setMotorInverted(TurretConstants.MOTOR_INVERTED);
        if (FieldConstants.IS_REAL) {
            motor.setEncoder(encoder.getDeviceID(), TurretConstants.MOTOR_TO_ENCODER_GEAR_RATIO, TurretConstants.ENCODER_TO_MECH_GEAR_RATIO);
        }
        else {
            motor.setGearRatio(TurretConstants.MOTOR_GEAR_RATIO);
        }
        motor.setGains(TurretConstants.GAINS);
        motor.setStatorCurrentLimit(TurretConstants.CURRENT_LIMIT);
        motor.setTorqueCurrentLimits(-TurretConstants.CURRENT_LIMIT, TurretConstants.CURRENT_LIMIT);
        motor.setBrakeMode(TurretConstants.BRAKE_MOTOR);
        motor.setTelemetryPreference(TelemetryPreference.DEFAULT);
        motor.setPositionClosedLoopWrappingEnabled(true);
    }

    private void configEncoder() {
        
        //v setting to inversion of motor so they stay the same(copying what was done in turret-test branch)
        encoder.configureMagnetSensor(TurretConstants.CANCODER_INVERTED, TurretConstants.ABSOLUTE_ENCODER_OFFSET, TurretConstants.DISCONTINUITY_POINT);
        encoder.setPositionConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(TurretConstants.VELOCITY_CONVERSION_FACTOR);
    }

    /**
     * Updates the inputs sent to the turret motor
     */
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Call refreshALl() to refresh all status signals, and check in on him :)
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.encoderConnected = encoder.refreshSignals().isOK();
        if (inputs.encoderConnected) {
            inputs.encoderAbsPositionRads = encoder.getPositionAsDouble();
        //     if (!encoderUpdated) {
        //         motor.resetEncoder(
        //             inputs.encoderAbsPositionRads * (TurretConstants.MOTOR_GEAR_RATIO / TurretConstants.ENCODER_GEAR_RATIO) > TurretConstants.MAX_ANGLE_RADIANS 
        //                 ? inputs.encoderAbsPositionRads * (TurretConstants.MOTOR_GEAR_RATIO / TurretConstants.ENCODER_GEAR_RATIO) - 2 * Math.PI 
        //                 : inputs.encoderAbsPositionRads * (TurretConstants.MOTOR_GEAR_RATIO / TurretConstants.ENCODER_GEAR_RATIO));
        //         encoderUpdated = true;
        //     }
        // } else {
        //     inputs.encoderAbsPositionRads = inputs.internalPositionRads;
        }   
        inputs.internalPositionRads = motor.getPositionAsDouble();
        inputs.internalVelocityRadsPerSec = motor.getVelocityAsDouble();
        inputs.targetPositionRads = motor.getTargetPosition();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setTargetPosition(position, feedforward);
    }

    @Override
    public void setNeutral() {
        motor.setNeutral();
    }

    @Override
    public void runCharacterization(double input) {
        motor.setVoltage(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setBrakeMode(brake);
    }

    @Override
    public void setGains(GainConstants constants) {
        motor.setGains(constants.withG(0.0));
    }

    public void resetEncoder() {
        motor.resetEncoder(0.0);
        encoder.resetEncoder(0.0);
    }
}

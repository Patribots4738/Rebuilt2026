package frc.robot.subsystems.superstructure.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.KickerConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;
import org.wpilib.units.measure.Distance;

public class KickerIOKraken implements KickerIO {

    private final Kraken motor;
    private final CANrange range;

    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<Boolean> isTripped;
    private final StatusSignal<Distance> CANRangeDistance;

    public KickerIOKraken() {
        motor = new Kraken(KickerConstants.CAN_ID, KickerConstants.MOTOR_TYPE, CANConstants.RIO_BUS,true, true, ControlPreference.TORQUE_CURRENT);
        var canRangeConfigs = new CANrangeConfiguration();
        //TODO jack in the bot numbers
        canRangeConfigs.ProximityParams.ProximityHysteresis = 0.01;
        canRangeConfigs.ProximityParams.ProximityThreshold = 0.07;
        canRangeConfigs.ProximityParams.MinSignalStrengthForValidMeasurement = 8000;
        range = new CANrange(KickerConstants.CANRANGE_ID, CANConstants.RIO_BUS);
        configMotor();

        signalStrength = range.getSignalStrength();
        isTripped = range.getIsDetected();
        CANRangeDistance = range.getDistance();
    }

    private void configMotor() {
        motor.setMotorInverted(KickerConstants.MOTOR_INVERTED);
        motor.setGearRatio(KickerConstants.GEAR_RATIO);
        motor.setSupplyCurrentLimit(KickerConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(KickerConstants.CURRENT_LIMIT);
        setBrakeMode(KickerConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocityAsDouble();
        inputs.positionRotations = motor.getPositionAsDouble();
        
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.isRangeTripped = isTripped.getValue();
        inputs.rangeDistance = CANRangeDistance.getValueAsDouble();
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

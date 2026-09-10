package frc.robot.subsystems.superstructure.shooter.flywheel;

import frc.robot.util.Constants.CANConstants;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class FlywheelIOKraken implements FlywheelIO {
    
    private final Kraken leader;
    private final Kraken follower;


    public FlywheelIOKraken() {
        leader = new Kraken(FlywheelConstants.LEADER_CAN_ID, FlywheelConstants.MOTOR_TYPE, CANConstants.RIO_BUS,true, true, ControlPreference.VOLTAGE);
        follower = new Kraken(FlywheelConstants.FOLLOWER_CAN_ID, FlywheelConstants.MOTOR_TYPE, CANConstants.RIO_BUS,true, true, ControlPreference.VOLTAGE);
        configMotor(leader);
        configMotor(follower);
    }

    private void configMotor(Kraken motor) {
        motor.setMotorInverted(FlywheelConstants.MOTOR_INVERTED);
        motor.setSupplyCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
        motor.setGains(FlywheelConstants.GAINS);
        setBrakeMode(FlywheelConstants.BRAKE_MOTOR);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leaderConnected = leader.refreshSignals().isOK();
        inputs.leaderPercentOutput = leader.getPercentAsDouble();
        inputs.leaderTargetSpeedRPM = leader.getTargetVelocity() * 60;
        inputs.leaderAppliedOutputVolts = leader.getVoltageAsDouble();
        inputs.leaderTorqueCurrentAmps = leader.getTorqueCurrentAsDouble();
        inputs.leaderTemperatureCelsius = leader.getTemperatureAsDouble();
        inputs.leaderPositionRadians = leader.getPositionAsDouble() * 2.0 * Math.PI;
        inputs.leaderSpeedRPM = leader.getVelocityAsDouble() * 60;

        inputs.followerConnected = follower.refreshSignals().isOK();
        inputs.followerPercentOutput = follower.getPercentAsDouble();
        inputs.followerTargetSpeedRPM = follower.getTargetVelocity() * 60;
        inputs.followerAppliedOutputVolts = follower.getVoltageAsDouble();
        inputs.followerTorqueCurrentAmps = follower.getTorqueCurrentAsDouble();
        inputs.followerTemperatureCelsius = follower.getTemperatureAsDouble();
        inputs.followerPositionRadians = follower.getPositionAsDouble() * 2.0 * Math.PI;
        inputs.followerSpeedRPM = follower.getVelocityAsDouble() * 60;
    }
    
    @Override
    public void setNeutral() {
        leader.setNeutral();
        follower.setNeutral();
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        follower.setBrakeMode(brake);
    }

    @Override
    public void setSpeedRPM(double speed) {
        leader.setTargetVelocity(speed / 60);
        follower.setTargetVelocity(speed / 60);
    }

    @Override
    public void setPercent(double percent) {
        leader.setPercentOutput(percent);
        follower.setPercentOutput(percent);
    }

    @Override
    public void setVoltage(double volts) {
        leader.setVoltageOutput(volts);
        follower.setVoltageOutput(volts);
    }

    @Override
    public void setCurrent(double amps) {
        leader.setTorqueCurrentOutput(amps);
        follower.setTorqueCurrentOutput(amps);
    }

    @Override
    public void setGains(GainConstants constants) {
        leader.setGains(constants.withG(0.0));
        follower.setGains(constants.withG(0.0));
    }
}

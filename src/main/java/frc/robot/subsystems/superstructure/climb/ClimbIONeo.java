package frc.robot.subsystems.superstructure.climb;

import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.custom.GainConstants;
import frc.robot.util.hardware.rev.Neo;

public class ClimbIONeo implements ClimbIO {
    private final Neo motor;

    public ElevatorIONeo() {
        motor = new Neo(ClimbConstants.CAN_ID, false);
        configMotor();
    }

    private void configMotor(Neo motor) {
        motor.setPID(ClimbConstants.GAINS);
        motor.setInverted(ClimbConstants.MOTOR_INVERTED);
        motor.setPositionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR);
        motor.setVelocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR / 60.0);
        motor.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderMotorConnected = true;
        inputs.leaderPositionMeters = leader.getPosition();
        inputs.leaderTargetPositionMeters = leader.getTargetPosition();
        inputs.leaderVelocityMetersPerSecond = leader.getVelocity();
        inputs.leaderAppliedOutputVolts = leader.getBusVoltage();
        inputs.leaderTorqueCurrentAmps = leader.getOutputCurrent();
        inputs.leaderTemperatureCelsius = leader.getMotorTemperature();

        inputs.followerMotorConnected = true;
        inputs.followerPositionMeters = follower.getPosition();
        inputs.followerTargetPositionMeters = follower.getTargetPosition();
        inputs.followerVelocityMetersPerSecond = follower.getVelocity();
        inputs.followerAppliedOutputVolts = follower.getBusVoltage();
        inputs.leaderTorqueCurrentAmps = follower.getOutputCurrent();
        inputs.followerTemperatureCelsius = follower.getMotorTemperature();
    }

    @Override
    public void setPosition(double position) {
        leader.setTargetPosition(position);
        follower.setTargetPosition(position);
    }

    @Override
    public void runCharacterization(double input) {
        leader.setVoltage(input);
        follower.setVoltage(input);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            leader.setBrakeMode();
            follower.setBrakeMode();
        } else {
            leader.setCoastMode();
            follower.setCoastMode();
        }
    }

    @Override
    public void resetEncoders(double position) {
        leader.resetEncoder(position);
        follower.resetEncoder(position);
    }

    @Override
    public void setGains(GainConstants constants) {
        leader.setPID(constants);
        follower.setPID(constants);
    }
}

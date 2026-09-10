package frc.robot.subsystems.superstructure.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {

        public boolean leaderConnected = false;
        public double leaderPercentOutput = 0.0;
        public double leaderTargetSpeedRPM = 0.0;
        public double leaderAppliedOutputVolts = 0.0;
        public double leaderTorqueCurrentAmps = 0.0;
        public double leaderTemperatureCelsius = 0.0;
        public double leaderPositionRadians = 0.0;
        public double leaderSpeedRPM = 0.0;

        public boolean followerConnected = false;
        public double followerPercentOutput = 0.0;
        public double followerTargetSpeedRPM = 0.0;
        public double followerAppliedOutputVolts = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerTemperatureCelsius = 0.0;
        public double followerPositionRadians = 0.0;
        public double followerSpeedRPM = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setSpeedRPM(double speed) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}

    public default void setGains(GainConstants constants) {}
}

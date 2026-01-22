package frc.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface ClimbIO {
    
    @AutoLog
    class ClimbIOInputs {
        public boolean motorConnected = false;
        public double positionMeters = 0.0;
        public double targetPositionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    default void updateInputs(ClimbIOInputs inputs) {}

    default void setNeutral() {}

    default void setPosition(double position) {}

    default void runCharacterization(double input) {}

    default void setBrakeMode(boolean brake) {}

    default void resetEncoder(double position) {}

    default void setGains(GainConstants constants) {}

    default void configureProfile(double velocity, double acceleration, double jerk) {}

}

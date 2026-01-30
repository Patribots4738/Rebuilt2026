package frc.robot.subsystems.superstructure.kicker;

import frc.robot.util.custom.GainConstants;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    
    @AutoLog
    class KickerIOInputs {
        public boolean motorConnected = false;
        public double motorPositionMeters;
        public double motorTargetPositionMeters;
        public double motorVelocityMetersPerSecond;
        public double motorAppliedOutputVolts;
        public double motorTorqueCurrentAmps;
        public double motorTemperatureCelsius;
        public double positionRads;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void resetEncoders(double position) {}

    public default void setGains(GainConstants constants) {}

    public default void configureProfile(double velocity, double acceleration, double jerk) {}

}
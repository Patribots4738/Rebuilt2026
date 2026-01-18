package frc.robot.subsystems.superstructure.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface HoodIO {

    @AutoLog
    class HoodIOInputs {
        public boolean motorConnected = false;
        public double internalPositionRads = 0.0;
        public double internalVelocityRadsPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean encoderConnected = false;
        public double encoderAbsPositionRads = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setPosition(double position, double feedforward) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}
        
    public default void setGains(GainConstants constants) {}

    public default void configureProfile(double velocity, double acceleration, double jerk) {}
}

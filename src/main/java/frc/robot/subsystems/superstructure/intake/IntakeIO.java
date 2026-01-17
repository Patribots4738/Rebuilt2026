package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface IntakeIO {
    
    @AutoLog
    class IntakeIOInputs {
        public boolean MotorConnected = false;
        public double motorPositionMeters = 0.0;
        public double motorTargetPositionMeters = 0.0;
        public double motorVelocityMetersPerSecond = 0.0;
        public double motorAppliedOutputVolts = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTemperatureCelsius = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void resetEncoders(double position) {}

    public default void setGains(GainConstants constants) {}

    public default void configureProfile(double velocity, double acceleration, double jerk) {}

}
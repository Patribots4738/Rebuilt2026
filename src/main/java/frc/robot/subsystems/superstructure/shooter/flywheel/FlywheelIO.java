package frc.robot.subsystems.superstructure.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {

        public boolean motorConnected = false;
        public double percentOutput = 0.0;
        // TODO: these should all be RPM, not rad (rad only for hood/turret)
        public double targetVelocityRadsPerSec = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double positionRotations = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setNeutral() {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setVelocity(double velocity) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}
}

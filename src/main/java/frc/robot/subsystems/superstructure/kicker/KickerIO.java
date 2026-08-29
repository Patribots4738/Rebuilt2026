package frc.robot.subsystems.superstructure.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    
    @AutoLog
    class KickerIOInputs {
        public boolean motorConnected = false;
        public double percentOutput  = 0.0;
        public double targetPercentOutput = 0.0;
        public double velocityRotationsPerSec = 0.0;    
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double positionRotations = 0.0;
        public boolean isRangeTripped = true;
        public double rangeDistance = 0.0;
        public double signalStrength = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setNeutral() {}

    public default void setBrakeMode(boolean brake) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}
  
    public default void updateCANRange() {}

}

package frc.robot.subsystems.superstructure.flywheel;

import frc.robot.util.custom.GainConstants;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    
    @AutoLog
    class FlywheelIOInputs {
        public boolean motorConnected = false;
        public double percentOutput  = 0.0;
        public double targetPercentOutput = 0.0;
        public double velocityRotationsPerSec = 0.0;    
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double positionRotations = 0.0;
        public boolean isRangeTripped;
        public double rangeDistance;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setNeutral() {}

    public default void setBrakeMode(boolean brake) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}
  
    public default void updateCANRange() {}

}
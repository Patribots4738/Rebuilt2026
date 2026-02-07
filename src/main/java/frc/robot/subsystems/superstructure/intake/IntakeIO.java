package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    class IntakeIOInputs {
        public boolean motorConnected = false;
        public double percentOutput  = 0.0;
        public double targetPercentOutput = 0.0;
        public double velocityRotationsPerSec = 0.0;    
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setNeutral() {}
    
    public default void setBrakeMode(boolean brake) {}
    
    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}

}
package frc.robot.subsystems.spindexer;
import org.littletonrobotics.junction.AutoLog;
public interface SpindexerIO {
    
    @AutoLog
    class SpindexerIOInputs {
        public boolean motorConnected = false;
        public double internalPositionRads = 0.0;
        public double internalVelocityRadsPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelcius = 0.0;
        public double percentOutput = 0.0;
        public double targetPercentOutput = 0.0;

    }

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void setNeutral() {}

    public default void setBrakeMode(boolean brake) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}
}

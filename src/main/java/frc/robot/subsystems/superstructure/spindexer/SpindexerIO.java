package frc.robot.subsystems.superstructure.spindexer;
import org.littletonrobotics.junction.AutoLog;
public interface SpindexerIO {
    
    @AutoLog
    class SpindexerIOInputs {
        public boolean motorConnected = false;
        public double internalVelocityRadsPerSec = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double percentOutput = 0.0;
        public double targetPercentOutput = 0.0;
        public double velocityRotationsPerSec = 0.0;
    }

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void setNeutral() {}

    public default void setBrakeMode(boolean brake) {}

    public default void runCharacterization(double input) {}

    public default void resetEncoders(double position) {}

    public default void setPercent(double percent) {}

    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}

}

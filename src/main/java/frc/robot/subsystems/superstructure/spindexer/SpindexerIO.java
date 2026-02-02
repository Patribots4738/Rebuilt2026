package frc.robot.subsystems.superstructure.spindexer;
import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;
public interface SpindexerIO {
    
    @AutoLog
    class SpindexerIOInputs {
        //It looks like none of the variables need to actually be initialized, so
        //maybe consider removing the definitions?
        public boolean motorConnected = false;
        public double internalPositionRads = 0.0;
        public double internalVelocityRadsPerSec = 0.0;
        public double targetPositionRads = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double percentOutput = 0.0;
        public double targetPercentOutput = 0.0;
        public double velocityRotationsPerSec;
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

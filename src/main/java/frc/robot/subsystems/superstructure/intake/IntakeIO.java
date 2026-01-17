package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface IntakeIO {
    
    @AutoLog
    class IntakeIOInputs {
        
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
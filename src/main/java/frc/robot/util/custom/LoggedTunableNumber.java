package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.internal.DriverStationBackend;

public class LoggedTunableNumber extends LoggedNetworkNumber {

    private double previousValue;
    
    public LoggedTunableNumber(String key, double defaultValue) {
        super(key, defaultValue);
        this.previousValue = defaultValue;
    }

    public LoggedTunableNumber(String key) {
        this(key, 0.0);
    }

    public boolean ifChanged() {
        return get() != previousValue && !DriverStationBackend.isFMSAttached();
    }

    public Trigger onChanged() {
        return new Trigger(this::ifChanged);
    }

    public Trigger onChanged(Command command) {
        return onChanged().onTrue(command);
    }
    
    @Override
    public void periodic() {
        previousValue = get();
        super.periodic();
    }
}

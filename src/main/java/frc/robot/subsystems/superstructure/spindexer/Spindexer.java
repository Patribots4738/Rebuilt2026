package frc.robot.subsystems.superstructure.spindexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {

    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private double testPercent;
	
	public Spindexer(SpindexerIO io) {
		this.io = io;

        SpindexerConstants.TEST_PERCENT.onChanged(Commands.runOnce(() -> testPercent = SpindexerConstants.TEST_PERCENT.get()).ignoringDisable(true));
	}
	
	@Override
    public void periodic(){
        io.updateInputs(inputs);

        if (shouldRunSetpoint) {
            io.setPercent(percentOutput);
        } else {
            io.setNeutral();
        }
    }

    private double percentOutput = 0.0;
    private boolean shouldRunSetpoint = false;

    public void setPercent(double percent) {
        percentOutput = percent;
        shouldRunSetpoint = true;
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public void setTestPercent() {
        this.setPercent(testPercent);
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
    }

    public Command setTestPercentCommand() {
        return Commands.runOnce(() -> setTestPercent());
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }
    
    public double getVelocity() {
        return inputs.velocityRotationsPerSec;
    }
    
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
}

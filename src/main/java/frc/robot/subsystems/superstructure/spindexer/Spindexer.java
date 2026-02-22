package frc.robot.subsystems.superstructure.spindexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Constants.SpindexerConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Spindexer extends SubsystemBase {

    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    
    private final SysIdRoutine spindexerRoutine;
	
	public Spindexer(SpindexerIO io) {
		this.io = io;
        this.spindexerRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.setVoltage(volts.in(Volts)),
                        log -> log.motor("spindexer")
                                .voltage(Volts.of(inputs.appliedOutputVolts))
                                .angularPosition(Rotations.of(inputs.positionRotations))
                                .angularVelocity(RotationsPerSecond.of(inputs.velocityRotationsPerSec * SpindexerConstants.SPINDEXER_RADIUS * 2 * Math.PI)),
                        this
                )
        );
	}
	//Controls the amount of power and rotation/speed of the spindexer
	@Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Spindexer", inputs);
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

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
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
    
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return spindexerRoutine.quasistatic(direction);
    }
    
    public Command dynamic(SysIdRoutine.Direction direction) {
        return spindexerRoutine.dynamic(direction);
    }
}

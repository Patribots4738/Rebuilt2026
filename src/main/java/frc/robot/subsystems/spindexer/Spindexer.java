package frc.robot.subsystems.spindexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.util.Constants.SpindexerConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.LoggingConstants.Mode;

public class Spindexer extends SubsystemBase {

    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    @Override
    public void periodic(){
        io.updateInputs(inputs);
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
}

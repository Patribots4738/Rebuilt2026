package frc.robot.subsystems.superstructure.shooter.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.calc.ShotCalculations;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {
    
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    
    private final SysIdRoutine flywheelRoutine;

    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/TargetLinearVelocity")
    private double targetVelocity = 0.0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        //NOTE TO SELF CAUSE IM GONNA FORGET: i put this here since itll have to be construnted every time a new flywheel is made cause if it was in the other one it would like uhh make a new one every time it was called probably so im very very smart for this (maybe (maybe not))
        this.flywheelRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
                        log -> log.motor("flywheel")
                                .voltage(Volts.of(inputs.appliedOutputVolts))
                                .linearVelocity(MetersPerSecond.of(inputs.velocityRadsPerSec)),
                        this
                )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Flywheel", inputs);

        // Run setpoint on RIO to minimize CAN utilization
        if (shouldRunSetpoint) {
            io.setVelocity(targetVelocity);
        } else {
            io.setNeutral();
        }
    }

    
    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    /**
     * Sets the velocity of the wheel using the desired tangential velocity
     * 
     * @param velocity tangential velocity
     */
    public void setVelocity(double velocity) {
        targetVelocity = velocity / FlywheelConstants.CIRCUMFERENCE_METERS;
        shouldRunSetpoint = true;
    }

    public boolean atTargetVelocity() {
        return atVelocity(targetVelocity);
    }

    public boolean atVelocity(double velocity) {
        return MathUtil.isNear(getVelocity(), velocity, FlywheelConstants.VELOCITY_DEADBAND_METERS_PER_SEC);
    }

    /**
     * Returns the tangential velocity of the flywheel
     * 
     * @return the tangential velocity of the flywheel
     */
    public double getVelocity() {
        return inputs.velocityRadsPerSec * FlywheelConstants.CIRCUMFERENCE_METERS;
    }

    public double getTargetVelocity() {
        return targetVelocity * FlywheelConstants.CIRCUMFERENCE_METERS;
    }

    public Command setVelocityCommand(DoubleSupplier velocitySupplier) {
        return runOnce(() -> setVelocity(velocitySupplier.getAsDouble()));
    }

    public Command setVelocityCommand(double velocity) {
        return setVelocityCommand(() -> velocity);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }
    
    public Command runCharacterization(double volts) {
        return runOnce(() -> io.runCharacterization(volts));
    }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }
    
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return flywheelRoutine.quasistatic(direction);
    }
    
    public Command dynamic(SysIdRoutine.Direction direction) {
        return flywheelRoutine.dynamic(direction);
    }
}

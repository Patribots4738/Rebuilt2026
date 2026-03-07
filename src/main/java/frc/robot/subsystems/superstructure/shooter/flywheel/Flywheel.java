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
import edu.wpi.first.wpilibj2.command.Commands;
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
    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/TargetSpeedRPM")
    private double targetSpeed = 0.0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        // NOTE TO SELF CAUSE IM GONNA FORGET: i put this here since itll have to be
        // constructed every time a new flywheel is made cause if it was in the other one
        // it would like uhh make a new one every time it was called probably so im very
        // very smart for this (maybe (maybe not))

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
                                .angularPosition(Rotations.of(inputs.positionRotations))
                                .angularVelocity(RadiansPerSecond.of(inputs.speedRPM / 60)),
                        this
                )
        );

        FlywheelConstants.LOGGED_FLYWHEEL_GAINS.onChanged(Commands.runOnce(() -> io.setGains(FlywheelConstants.LOGGED_FLYWHEEL_GAINS.get())).ignoringDisable(true));
        FlywheelConstants.LOGGED_LINEAR_VELOCITY.onChanged(Commands.runOnce(() -> testLinearVelocity = FlywheelConstants.LOGGED_LINEAR_VELOCITY.get()).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Flywheel", inputs);

        if (shouldRunSetpoint) {
            io.setSpeedRPM(targetSpeed);
        } else {
            io.setNeutral();
        }
    }

    
    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    /**
     * Sets the desired speed of the flywheel
     * 
     * @param speed desired speed in RPM
     */
    public void setSpeedRPM(double speed) {
        targetSpeed = speed;
        shouldRunSetpoint = true;
    }

    public boolean atTargetSpeed() {
        return atSpeed(targetSpeed);
    }

    public boolean atSpeed(double speed) {
        return MathUtil.isNear(getSpeedRPM(), speed, FlywheelConstants.VELOCITY_DEADBAND_RPM); 
    }

    /**
     * Returns the speed of the flywheel
     * 
     * @return speed RPM
     */
    public double getSpeedRPM() {
        return inputs.speedRPM;
    }

    public double getTargetSpeedRPM() {
        return targetSpeed; 
    }

    public Command setSpeedRPMCommand(DoubleSupplier speedSupplier) {
        return runOnce(() -> setSpeedRPM(speedSupplier.getAsDouble()));
    }

    public Command setSpeedRPMCommand(double speed) {
        return setSpeedRPMCommand(() -> speed);
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

package frc.robot.subsystems.superstructure.shooter.flywheel;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Constants.FlywheelConstants;
import frc.robot.util.Constants.LoggingConstants;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {
    
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    
    // private final SysIdRoutine flywheelRoutine;

    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    @AutoLogOutput (key = "Subsystems/Shooter/Flywheel/TargetSpeedRPM")
    private double targetSpeed = 0.0;
    // private double calculatedSpeed = 0.0;
    private double incrementedSpeed = 4000.0;

    // private double testSpeed;
    // private boolean isCharacterizing = false;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        // NOTE TO SELF CAUSE IM GONNA FORGET: i put this here since itll have to be
        // constructed every time a new flywheel is made cause if it was in the other one
        // it would like uhh make a new one every time it was called probably so im very
        // very smart for this (maybe (maybe not))

        // this.flywheelRoutine = new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //                 Volts.per(Seconds).of(1),
        //                 Volts.of(7),
        //                 Seconds.of(10)
        //         ),
        //         new SysIdRoutine.Mechanism(
        //                 (Voltage volts) -> io.setVoltage(volts.in(Volts)),
        //                 log -> log.motor("flywheel")
        //                         .voltage(Volts.of(inputs.appliedOutputVolts))
        //                         .angularPosition(Rotations.of(inputs.positionRadians))
        //                         .angularVelocity(RadiansPerSecond.of(inputs.speedRPM / 60)),
        //                 this
        //         )
        // );

        // FlywheelConstants.LOGGED_FLYWHEEL_GAINS.onChanged(Commands.runOnce(() -> io.setGains(FlywheelConstants.LOGGED_FLYWHEEL_GAINS.get())).ignoringDisable(true));
        // FlywheelConstants.LOGGED_SPEED_RPM.onChanged(Commands.runOnce(() -> testSpeed = FlywheelConstants.LOGGED_SPEED_RPM.get()).ignoringDisable(true));
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

    public boolean atTargetSpeed(BooleanSupplier aimingAtHub) {
        return atTargetSpeed(aimingAtHub.getAsBoolean());
    }

    public boolean atTargetSpeed(boolean aimingAtHub) {
        return atSpeed(targetSpeed, aimingAtHub);
    }

    public boolean atSpeed(double speed, boolean aimingAtHub) {
        return MathUtil.isNear(getSpeedRPM(), speed, aimingAtHub 
                ? FlywheelConstants.VELOCITY_DEADBAND_RPM 
                : FlywheelConstants.PASS_VELOCITY_DEADBAND_RPM); 
    }

    /**
     * Returns the speed of the flywheel
     * 
     * @return speed RPM
     */
    public double getSpeedRPM() {
        return inputs.speedRPM;
    }

    private void decrementSpeed() {
        incrementedSpeed -= 50;
    }

    private void incrementSpeed() {
        incrementedSpeed += 50;
    }

    public Command decrementSpeedCommand() {
        return runOnce(() -> decrementSpeed());
    }

    public Command incrementSpeedCommand() {
        return runOnce(() -> incrementSpeed());
    }

    public Command setIncrementSpeedCommand() {
        return setSpeedRPMCommand(() -> incrementedSpeed);
    }

    // public void setCalculatedSpeedRPM(double speed) {
    //     calculatedSpeed = speed;
    // }

    // public Command setCalculatedSpeedRPMCommand(double speed) {
    //     return runOnce(() -> setCalculatedSpeedRPM(speed));
    // }

    // public Command setCalculatedSpeedRPMCommand(DoubleSupplier speed) {
    //     return runOnce(() -> setCalculatedSpeedRPM(speed.getAsDouble()));
    // }

    // public void runCalculatedSpeed() {
    //     setSpeedRPM(calculatedSpeed);
    // }

    // public Command runCalculateSpeedCommand() {
    //     return setSpeedRPMCommand(() -> calculatedSpeed);
    // }

    public double getTargetSpeedRPM() {
        return targetSpeed; 
    }

    public Command setSpeedRPMCommand(DoubleSupplier speedSupplier) {
        return run(() -> setSpeedRPM(speedSupplier.getAsDouble()));
    }

    public Command setSpeedRPMCommand(double speed) {
        return setSpeedRPMCommand(() -> speed);
    }

    // public Command setTestSpeedRPMCommand() {
    //     return setSpeedRPMCommand(() -> testSpeed);
    // }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public void stop() {
        setNeutral();
        targetSpeed = 0;
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
    
    public Command runCharacterization(double volts) {
        return runOnce(() -> io.runCharacterization(volts));
    }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }
    
    // public Command quasistatic(SysIdRoutine.Direction direction) {
    //     return flywheelRoutine.quasistatic(direction)
    //             .beforeStarting(() -> isCharacterizing = true)
    //             .finallyDo(interrupted -> {
    //                 isCharacterizing = false;
    //                 setNeutral(); // optional safety
    //             });
    // }
    
    // public Command dynamic(SysIdRoutine.Direction direction) {
    //     return flywheelRoutine.dynamic(direction)
    //             .beforeStarting(() -> isCharacterizing = true)
    //             .finallyDo(interrupted -> {
    //                 isCharacterizing = false;
    //                 setNeutral(); // optional safety
    //             });
    // }
}

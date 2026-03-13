package frc.robot.subsystems.superstructure.shooter.hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import frc.robot.util.calc.ShotCalculations;

import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    @AutoLogOutput (key = "Subsystems/Shooter/Hood/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    private double percentOutput = 0.0;
    
    private double targetPosition;
    private double testPosition;

	private final SysIdRoutine hoodRoutine;
	
    public Hood(HoodIO io) {
        this.io = io;

        HoodConstants.LOGGED_HOOD_GAINS.onChanged(Commands.runOnce(() -> io.setGains(HoodConstants.LOGGED_HOOD_GAINS.get())).ignoringDisable(true));
        HoodConstants.LOGGED_TEST_POSITION_DEGREES.onChanged(Commands.runOnce(() -> testPosition = HoodConstants.LOGGED_TEST_POSITION_DEGREES.get()).ignoringDisable(true));

        this.setDefaultCommand(this.setNeutralCommand());
        this.hoodRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(1),
                        Volts.of(7),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
                        log -> log.motor("hood")
                                .voltage(Volts.of(inputs.appliedOutputVolts))
                                .angularPosition(Radians.of(inputs.internalPositionRads))
                                .angularVelocity(RadiansPerSecond.of(inputs.internalVelocityRadsPerSec)),
                        this
                )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Hood", inputs);

        Logger.recordOutput("Subsystems/Shooter/Hood/AtTargetPosition", atTargetPosition());
        Logger.recordOutput("Subsystems/Shooter/Hood/TargetPosition", targetPosition);

        // if (shouldRunSetpoint) {
            io.setPosition(targetPosition, 0);
        // } else {
        //     io.setNeutral();
        // }

        // RobotContainer.hoodMechanism.setAngle(inputs.internalPositionRads);

        RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            LoggingConstants.HOOD_OFFSET.getX(),
            LoggingConstants.HOOD_OFFSET.getY(),
            LoggingConstants.HOOD_OFFSET.getZ(),
            new Rotation3d(0.0, 
                inputs.internalPositionRads, 
                RobotContainer.components3d[LoggingConstants.TURRET_INDEX].getRotation().getAngle())
        );
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position
     */
    public void setPosition(double position) {
        position = MathUtil.clamp(position, HoodConstants.MIN_ANGLE_RADIANS, HoodConstants.MAX_ANGLE_RADIANS);
        targetPosition = position;
        shouldRunSetpoint = true;
        // io.setPosition(targetPosition, 0);

        RobotContainer.desiredComponents3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
            RobotContainer.desiredComponents3d[LoggingConstants.HOOD_INDEX].getX(), 
            RobotContainer.desiredComponents3d[LoggingConstants.HOOD_INDEX].getY(),
            RobotContainer.desiredComponents3d[LoggingConstants.HOOD_INDEX].getZ(),
            new Rotation3d(0, position, 0)
        );

        // RobotContainer.hoodMechanism.setAngle(position);    
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier) {
        return runOnce(() -> setPosition(positionSupplier.getAsDouble()));
    }

    public Command setPositionCommand(double position) {
        return setPositionCommand(() -> position);
    }

    public void setTestPosition() {
        setPosition(testPosition);
    }

    public Command setTestPositionCommand() {
        return Commands.runOnce(() -> setTestPosition());
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public Command resetEncodersCommand() {
        return runOnce(() -> io.resetEncoders(0));
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.internalPositionRads, HoodConstants.POSITION_DEADBAND_RADIANS);
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public double getPosition() {
        return inputs.internalPositionRads;
    }

    public double getVelocity() {
        return inputs.internalVelocityRadsPerSec;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setPercent(double percent) {
        percentOutput = percent;
        shouldRunSetpoint = true;
    }

    public Command setPercentCommand(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
    return Commands.runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public double getCharacterizationVelocity() {
        return inputs.internalVelocityRadsPerSec / HoodConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    public void setCurrent(double amps) {
        io.setCurrent(amps);
    }

    public Command setCurrentCommand(double amps) {
        return runOnce(() -> setCurrent(amps));
    }
    
    // public Command quasistatic(SysIdRoutine.Direction direction) {
    //     return hoodRoutine.quasistatic(direction);
    // }
    
    // public Command dynamic(SysIdRoutine.Direction direction) {
    //     return hoodRoutine.dynamic(direction);
    // }
}

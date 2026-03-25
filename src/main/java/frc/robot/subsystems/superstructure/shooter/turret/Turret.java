package frc.robot.subsystems.superstructure.shooter.turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import frc.robot.util.Constants.TurretConstants;
import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Rotation2d targetPosition =  getRotation();
    @AutoLogOutput (key = "Subsystems/Shooter/Turret/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    
    // private double testPositionDegrees;

    // private final SysIdRoutine turretRoutine;


    public Turret(TurretIO io) {
        this.io = io;

        // TurretConstants.LOGGED_TURRET_GAINS.onChanged(Commands.runOnce(() -> io.setGains(TurretConstants.LOGGED_TURRET_GAINS.get())).ignoringDisable(true));
        // TurretConstants.LOGGED_TEST_POSITION_DEGREES.onChanged(Commands.runOnce(() -> testPositionDegrees = TurretConstants.LOGGED_TEST_POSITION_DEGREES.get()).ignoringDisable(true));

        // this.turretRoutine = new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //                 Volts.per(Seconds).of(1),
        //                 Volts.of(7),
        //                 Seconds.of(10)
        //         ),
        //         new SysIdRoutine.Mechanism(
        //                 (Voltage volts) -> io.runCharacterization(volts.in(Volts)),
        //                 log -> log.motor("turret")
        //                         .voltage(Volts.of(inputs.appliedOutputVolts))
        //                         .angularPosition(Radians.of(inputs.internalPositionRads))
        //                         .angularVelocity(RadiansPerSecond.of(inputs.internalVelocityRadsPerSec)),
        //                 this
        //         )
        // );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter/Turret", inputs);
        Logger.recordOutput("Subsystems/Shooter/Turret/TargetPositionDegrees", targetPosition.getDegrees());
        Logger.recordOutput("Subsystems/Shooter/Turret/CurrentPositionDegrees", getRotation().getDegrees());
        Logger.recordOutput("Subsystems/Shooter/Turret/ShotCalcRoboRelativePosition", targetPosition.getDegrees());
        if (shouldRunSetpoint) {
            io.setPosition(targetPosition.getRadians(), 0);
        }
        else {
            io.setNeutral();
        }
    }

    /**
     * Sets the desired rotation of the turret
     * 
     * @param position in radians 
     */
    public void setPosition(Rotation2d position) {
        Logger.recordOutput("Subsystems/Shooter/Turret/ShotCalcRoboRelativePosition", position.getDegrees());
            targetPosition = position.plus(Rotation2d.fromRadians(TurretConstants.ANGULAR_OFFSET));

            // if (targetPosition.getDegrees() < 0) {
            //     targetPosition = Rotation2d.fromDegrees(180).minus(Rotation2d.fromDegrees(Math.abs(targetPosition.getDegrees()))).plus(Rotation2d.fromDegrees(180));
            // }

            shouldRunSetpoint = true;
        //if (position.getRadians() < TurretConstants.MAX_ANGLE_RADIANS && position.getRadians() > TurretConstants.MIN_ANGLE_RADIANS) {
            // apply the turret angular offset
            //targetPosition = position.plus(Rotation2d.fromRadians(TurretConstants.ANGULAR_OFFSET));
            //shouldRunSetpoint = true;
        //}

        // RobotContainer.components3d[LoggingConstants.HOOD_INDEX] = new Pose3d(
        //     LoggingConstants.TURRET_OFFSET,
        //     new Rotation3d(0, 0, inputs.encoderAbsPositionRads)
        // );
    }

    public Command setPositionCommand(Supplier<Rotation2d> rotationSupplier) {
        return setPositionCommand(rotationSupplier.get());
    }

    public Command setPositionCommand(Rotation2d rotation) {
        return runOnce(() -> setPosition(rotation));
    }

    // public Command setTestPositionCommand() {
    //     return setPositionCommand(() -> Rotation2d.fromRadians(Units.degreesToRadians(testPositionDegrees)));
    // }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }

    public boolean atPosition(Rotation2d position, boolean aimingAtHub) {
        return MathUtil.isNear(position.getRadians(), getRotation().getRadians(), aimingAtHub 
                ? TurretConstants.ROTATION_TOLERANCE_RADIANS 
                : TurretConstants.PASS_ROTATION_TOLERANCE_RADIANS);
    }

    public boolean atTargetPosition(boolean aimingAtHub) {
        return atPosition(targetPosition, aimingAtHub);
    }

    public boolean atTargetPosition(BooleanSupplier aimingAtHub) {
        return atTargetPosition(aimingAtHub.getAsBoolean());
    }

    public double getMotorPosition() {
        return inputs.internalPositionRads;
    }

    public Rotation2d getRotation() {
        return new Rotation2d(inputs.internalPositionRads - TurretConstants.ANGULAR_OFFSET);
    }

    public double getVelocity() {
        return inputs.internalVelocityRadsPerSec;
    }

    public Rotation2d getTargetPosition() {
        return targetPosition.minus(Rotation2d.fromRadians(TurretConstants.ROBOT_TURRET_OFFSET));
    }
    
    public double getVoltage() {
        return inputs.appliedOutputVolts;
    }

    public double getCharacterizationVelocity() {
        return inputs.internalVelocityRadsPerSec / TurretConstants.VELOCITY_CONVERSION_FACTOR;
    }

    public void runCharacterization(double input) {
        io.runCharacterization(input);
    }

    // public Command sysIdQuasistaticForward() {
    //     return turretRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    // public Command sysIdQuasistaticReverse() {
    //     return turretRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    // }

    // public Command sysIdDynamicForward() {
    //     return turretRoutine.dynamic(SysIdRoutine.Direction.kForward);
    // }

    // public Command sysIdDynamicReverse() {
    //     return turretRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    // }
}
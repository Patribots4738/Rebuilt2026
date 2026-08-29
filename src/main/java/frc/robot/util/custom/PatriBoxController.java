package frc.robot.util.custom;

import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

import java.util.function.DoubleSupplier;

import static frc.robot.util.calc.OICalc.toCircle;

public class PatriBoxController extends CommandGamepad {

    private final double deadband;

    public PatriBoxController(int port, double deadband) {
        super(port);

        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return getLeftAxis().getX();
    }

    @Override
    
    public double getLeftY() {
        return getLeftAxis().getY();
    }

    @Override
    public double getRightX() {
        return getRightAxis().getX();
    }

    @Override
    public double getRightY() {
        return getRightAxis().getY();
    }

    public Translation2d getLeftAxis() {
        // The Y inverted because for some reason when you
        // go forward on the controller, it returns a negative value.
        // I don't like that.
        return toCircle(MathUtil.applyDeadband(super.getLeftX(), deadband),
                MathUtil.applyDeadband(-super.getLeftY(), deadband));
    }

    public Translation2d getRightAxis() {
        // The Y inverted because for some reason when you
        // go forward on the controller, it returns a negative value.
        // I don't like that.
        return toCircle(MathUtil.applyDeadband(super.getRightX(), deadband),
                MathUtil.applyDeadband(-super.getRightY(), deadband));
    }

    private double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(
                (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                        (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

    public Command setRumble(DoubleSupplier rumble) {
        return Commands.runOnce(() -> setRumble(rumble.getAsDouble()));
    }

    public Command setRumble(DoubleSupplier rumble, double duration) {
        return Commands.runOnce(() -> setRumble(rumble.getAsDouble()))
            .andThen(Commands.waitSeconds(duration))
            .andThen(Commands.runOnce(() -> setRumble(0)));
    }

    public void setRumble(double rumble) {
        if (Robot.gameMode == GameMode.TELEOP) {
            this.getHID().setRumble(RumbleType.LEFT_RUMBLE, rumble);
            this.getHID().setRumble(RumbleType.RIGHT_RUMBLE, rumble);
        } else if (Robot.gameMode == GameMode.DISABLED) {
            this.getHID().setRumble(RumbleType.LEFT_RUMBLE, 0);
            this.getHID().setRumble(RumbleType.RIGHT_RUMBLE, 0);
        }
    }
}

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.hood.HoodIOKraken;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;

public class Shooter extends SubsystemBase {

    private boolean trackTarget = true;

    Turret turret;
    Hood hood;
    Flywheel flywheel;

    @AutoLogOutput(key = "")
    private Target target;

    Supplier<Pose2d> robotRelativePose;
    Supplier<ChassisSpeeds> robotRelativeVelocity;

    private enum Target {
        
    }

    public Shooter(Supplier<Pose2d> robotRelativePose, Supplier<ChassisSpeeds> robotRelativeVelocity) {
        this.robotRelativePose = robotRelativePose;
        this.robotRelativeVelocity = robotRelativeVelocity;

        turret = new Turret(new TurretIOKraken());
        hood = new Hood(new HoodIOKraken());
        flywheel = new Flywheel(new FlywheelIOKraken());
    }

    @Override
    public void periodic() {

    }
}

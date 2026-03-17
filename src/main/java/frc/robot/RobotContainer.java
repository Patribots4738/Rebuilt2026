package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.characterization.FeedForwardCharacterization;
import frc.robot.commands.characterization.StaticCharacterization;
import frc.robot.commands.characterization.WheelRadiusCharacterization;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.logging.NTGainTuner;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.ExtenderIOKraken;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIOKraken;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.kicker.KickerIOKraken;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.hood.HoodIOKraken;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.shooter.turret.TurretIOKraken;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.spindexer.SpindexerIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Constants.HoodConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.Alignment;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.custom.PatriBoxController;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.BooleanSupplier;

public class RobotContainer {

//     private PowerDistribution pdh;

//     @AutoLogOutput (key = "PowerDistributionHub/TotalCurrent")
//     private double totalCurrent;
//     @AutoLogOutput (key = "PowerDistributionHub/TotalPower")
//     private double totalPower;
//     @AutoLogOutput (key = "PowerDistributionHub/Voltage")
//     private double voltage;

    private EventLoop testButtonBindingLoop = new EventLoop();
    
    @AutoLogOutput (key = "Draggables/Components3d")
    public static Pose3d[] components3d = new Pose3d[5];
    @AutoLogOutput (key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;
    @SuppressWarnings("unused")
    private final Vision vision;
//     private final Climb climb;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final Intake intake;
    private final Extender extender;
    private final Kicker kicker;
    private final Spindexer spindexer;
    private final Alignment alignment;
    
    private final Shooter shooter;
    private final Superstructure superstructure;

    public static Field2d field2d = new Field2d();

    private PathPlannerStorage pathPlannerStorage;
    private static HDCTuner HDCTuner;

    // Draggables
    @AutoLogOutput (key = "Draggables/FreshCode")
    public static boolean freshCode = true;
    @AutoLogOutput (key = "Draggables/RobotPose2d")
    public static Pose2d robotPose2d = Pose2d.kZero;
    @AutoLogOutput (key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates;
    @AutoLogOutput (key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates;
    @AutoLogOutput (key = "Draggables/GameModeStart")
    public static double gameModeStart = 0;
    @AutoLogOutput (key = "Draggables/AutoStartingPose")
    public static Pose2d autoStartingPose = Pose2d.kZero;
    @AutoLogOutput(key = "Draggables/RobotPose3d")
    public static Pose3d robotPose3d = new Pose3d();
    public static Mechanism2d mech = new Mechanism2d(10, 10);
    public static MechanismRoot2d hoodPivot = mech.getRoot("Hood", 0, 0);
    public static MechanismLigament2d hoodMechanism = new MechanismLigament2d("hoodMechanism", 10, 0, 2, new Color8Bit(Color.kPurple));

    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        updateAllComponents3d();
        
        // pdh = new PowerDistribution(30, ModuleType.kRev);
        // pdh.setSwitchableChannel(false);

        // totalCurrent = pdh.getTotalCurrent();
        // SmartDashboard.putNumber("Total Current", totalCurrent);
        // totalPower = pdh.getTotalPower();
        // SmartDashboard.putNumber("Total Power", totalPower);
        // voltage = pdh.getVoltage();
        // SmartDashboard.putNumber("Voltage", voltage);

        swerve = new Swerve();
        alignment = new Alignment(swerve);
        vision = new Vision(swerve.getPoseEstimator(),
            new VisionIOLimelight("limelight-right", true),
            new VisionIOLimelight("limelight-left", true),
            new VisionIOLimelight("limelight-hailo", true),
            new VisionIOLimelight("limelight-threeg", false));
        // climb = new Climb(new ClimbIOKraken());
        extender = new Extender(new ExtenderIOKraken());
        intake = new Intake(new IntakeIOKraken());
        spindexer = new Spindexer(new SpindexerIOKraken());
        kicker = new Kicker(new KickerIOKraken());
        flywheel = new Flywheel(new FlywheelIOKraken());
        turret = new Turret(new TurretIOKraken());
        hood = new Hood(new HoodIOKraken());
        shooter = new Shooter(swerve::getPose, swerve::getRobotRelativeVelocity, turret, flywheel, hood);
        superstructure = new Superstructure(extender, intake, shooter, spindexer, kicker);

        SmartDashboard.putData(field2d);
        SmartDashboard.putData("Mech2d", mech);

        driver.back().toggleOnTrue(
                Commands.runOnce(() -> fieldRelativeToggle = !fieldRelativeToggle)
        );
        robotRelativeSupplier = () -> fieldRelativeToggle;

        swerve.setDefaultCommand(new Drive(
                swerve,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX() / 1.6,
                robotRelativeSupplier,
                () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        configureButtonBindings();
        configureTimedEvents();

        pathPlannerStorage = new PathPlannerStorage();

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
                swerve.setWheelsOCommand()
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(new WheelRadiusCharacterization(swerve)));
        pathPlannerStorage.getAutoChooser().addOption("DriveFeedForwardCharacterization",
                new FeedForwardCharacterization(
                        swerve,
                        swerve::runDriveCharacterization,
                        swerve::getDriveCharacterizationVelocity));
        pathPlannerStorage.getAutoChooser().addOption("TurnStaticCharacterization",
            new StaticCharacterization(
                swerve,
                swerve::runTurnCharacterization,
                swerve::getTurnCharacterizationVelocity));

        CommandScheduler.getInstance().schedule(new NTGainTuner());

        prepareNamedCommands();
    }

    private void configureButtonBindings() {
        switch(OIConstants.DRIVER_MODE) {
            case DEV:
                configureDevBindings(driver);
                break;
            default:
                configureDriverBindings(driver);
        }
    }

    private void configureTimedEvents() {
        // slow down the robot if we are aiming at the hub once the shooter is up to speed 
        // new Trigger(shooter::isReady && shooter::aimingAtHub)
        //         .onTrue(swerve.setDriveMultiplierCommand(DriveConstants.SHOOTING_DRIVE_MULTIPLIER))
        //         .onFalse(swerve.setDriveMultiplierCommand(DriveConstants.DEFAULT_DRIVE_MULTIPLIER));

    }

    private void configureDevBindings(PatriBoxController controller) {

        controller.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                        ? 0
                        : 180))
            ), swerve)
        );

        // controller.leftTrigger().onTrue(intake.setPercentCommand(IntakeConstants.INTAKE_PERCENT));
        // controller.leftBumper().onTrue(intake.setPercentCommand(0));

        // controller.rightBumper().onTrue(extender.setPositionCommand(ExtenderConstants.MAX_ANGLE_RADIANS));
        // controller.rightTrigger().onTrue(extender.setPositionCommand(ExtenderConstants.STOW_POSITION_RADIANS));

        // controller.leftTrigger().onTrue(Commands.parallel(
            //intake.setPercentCommand(IntakeConstants.INTAKE_PERCENT),
            // spindexer.setPercentCommand(SpindexerConstants.FEED_PERCENT),
            // kicker.setPercentCommand(KickerConstants.FORWARD_PERCENT)
        // ));

        // controller.leftBumper().onTrue(Commands.parallel(
        //     intake.setPercentCommand(0.0),
            // spindexer.setPercentCommand(0.0),
            // kicker.setPercentCommand(0.0)
        // ));

        // controller.x().onTrue(shooter.setManualStates());
        
        // controller.povUp().onTrue(flywheel.setSpeedRPMCommand(1000));
        // controller.povLeft().onTrue(flywheel.decrementSpeedCommand());
        // controller.povRight().onTrue(flywheel.incrementSpeedCommand());
        // controller.povDown().onTrue(flywheel.stopCommand());
        // controller.rightBumper().onTrue(flywheel.setIncrementSpeedCommand());
        
        // controller.x().onTrue(hood.setPositionCommand(0));
        // controller.b().onTrue(hood.setIncrementedPositionCommand());
        // controller.y().onTrue(hood.incrementPositionCommand());
        // controller.a().onTrue(hood.decrementPositionCommand());

        // controller.rightBumper().onTrue(extender.feedShooter());
        // controller.rightTrigger().onTrue(extender.setPositionCommand(ExtenderConstants.MAX_ANGLE_RADIANS));

        // controller.rightBumper().onTrue(hood.toggleForcedDownCommand());
        controller.a().onTrue(shooter.trackTargetCommand(true, true, true));
        controller.b().onTrue(shooter.trackTargetCommand(false, false, false));

        controller.povDown().onTrue(hood.setPositionCommand(0.0));
        // controller.rightTrigger().whileTrue(
        //     new ActiveConditionalCommand(
        //         superstructure.fireCommand(),
        //         Commands.sequence(
        //             shooter.armShooterCommand(),
        //             Commands.waitUntil(() -> shooter.isReadyToShoot()),
        //             superstructure.fireCommand()),
        //         () -> shooter.isReadyToShoot()
        //     )
        // );


        // controller.rightTrigger().toggleOnFalse(superstructure.ceaseFireCommand());
        

        //flywheel
        // controller.a().onTrue(flywheel.dynamic(SysIdRoutine.Direction.kForward));
        // controller.b().onTrue(flywheel.dynamic(SysIdRoutine.Direction.kReverse));

        // controller.x().onTrue(flywheel.quasistatic(SysIdRoutine.Direction.kForward));
        // controller.y().onTrue(flywheel.quasistatic(SysIdRoutine.Direction.kReverse));


        // swerve
        // controller.leftBumper().whileTrue(swerve.getSetWheelsX());
        // controller.rightBumper().whileTrue(swerve.getSetWheelsO());
        // controller.b().whileTrue(swerve.driveCharacterization());
        // controller.y().whileTrue(swerve.getSetWheelsZero());
    }

    private void configureDriverBindings(PatriBoxController controller) {

        controller.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                        ? 0
                        : 180))
            ), swerve)
        );

        // controller.leftTrigger().onTrue(superstructure.toggleIntakeCommand());
        // controller.leftBumper().onTrue(superstructure.stowClimbAndIntake());
        // controller.rightBumper().onTrue(hood.toggleForcedDownCommand());
        // controller.rightTrigger().whileTrue(
        //     new ActiveConditionalCommand(
        //         superstructure.fireCommand(),
        //         Commands.sequence(
        //             shooter.armShooterCommand(),
        //             Commands.waitUntil(() -> shooter.isReadyToShoot()),
        //             superstructure.fireCommand()),
        //         () -> shooter.isReadyToShoot()
        //     )
        // );
        // controller.rightTrigger().toggleOnFalse(superstructure.ceaseFireCommand());
        // // TODO: make these automatic based on position?
        // // you could have this as some sort of manual override, but seems unecesary with four cameras.
        // controller.rightStick().onTrue(shooter.setPassIndexCommand(1));
        // controller.leftStick().onTrue(shooter.setPassIndexCommand(-1));

        // controller.y().onTrue(shooter.toggleManualShootingCommand());

        // uncomment these when we have climb
        // controller.povUp().onTrue(superstructure.setClimbState(ClimbState.READY));
        // controller.povDown().and(controller.povRight().negate()).onTrue(superstructure.setClimbState(ClimbState.STOW));
        // controller.povDown().and(controller.povRight()).onTrue(superstructure.setClimbState(ClimbState.UP));
    }

    private void configureCalibrationBindings(PatriBoxController controller) {

    }

    public Command getAutonomousCommand() {
        return pathPlannerStorage.getSelectedAuto();
    }

    private void configureHDCBindings(PatriBoxController controller) {
        controller.pov(0, 270, testButtonBindingLoop)
                .onTrue(HDCTuner.controllerDecrementCommand());

        controller.pov(0, 90, testButtonBindingLoop)
                .onTrue(HDCTuner.controllerIncrementCommand());

        controller.pov(0, 0, testButtonBindingLoop)
                .onTrue(HDCTuner.increaseCurrentConstantCommand(.1));

        controller.pov(0, 180, testButtonBindingLoop)
                .onTrue(HDCTuner.increaseCurrentConstantCommand(-.1));

        controller.rightBumper(testButtonBindingLoop)
                .onTrue(HDCTuner.constantIncrementCommand());

        controller.leftBumper(testButtonBindingLoop)
                .onTrue(HDCTuner.constantDecrementCommand());

        controller.a(testButtonBindingLoop)
                .onTrue(HDCTuner.logCommand());

        controller.x(testButtonBindingLoop)
                .onTrue(HDCTuner.multiplyPIDCommand(2));

        controller.b(testButtonBindingLoop)
                .onTrue(HDCTuner.multiplyPIDCommand(.5));
    }

//     public void refreshPDHData() {
//         totalCurrent = pdh.getTotalCurrent();
//         totalPower = pdh.getTotalPower();
//         voltage = pdh.getVoltage();
//     }

    public void onDisabled() {
        // Stops bot in sim after auto (noramlly motors auto turn off when disabled)
        swerve.stopDriving();
        hood.setPosition(HoodConstants.STOW_POSITION_RADIANS);
        CommandScheduler.getInstance().schedule(pathPlannerStorage.updatePathViewerCommand());
        pathPlannerStorage.configureAutoChooser();
    }

    public void onEnabled() {
        gameModeStart = Robot.currentTimestamp;
        CommandScheduler.getInstance().schedule(pathPlannerStorage.updatePathViewerCommand());
        freshCode = false;
    }

    private void prepareNamedCommands() {
        // NamedCommands.registerCommand("Shoot", shooter.armShooterCommand().unless(() -> PoseCalculations.behindTower(swerve.getPose())));
        // NamedCommands.registerCommand("StopShooting", shooter.disarmShooterCommand());
        // NamedCommands.registerCommand("Track", shooter.trackTargetCommand(true, true, false));
        // NamedCommands.registerCommand("ClimbUp", superstructure.setClimbState(ClimbState.READY));
        // NamedCommands.registerCommand("ClimbDown", superstructure.setClimbState(ClimbState.UP));
        // NamedCommands.registerCommand("Intake", superstructure.setIntakeState(IntakeState.INTAKE));
        // NamedCommands.registerCommand("StopIntake", superstructure.setIntakeState(IntakeState.NO_SPIN));
        // NamedCommands.registerCommand("IntakeStow", superstructure.setIntakeState(IntakeState.STOW));

        NamedCommands.registerCommand("TowerAlignLeft", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), -1));
        NamedCommands.registerCommand("TowerAlignRight", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), 1));
        NamedCommands.registerCommand("TowerAlignCenter", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), 0));
    }
    
    private void updateAllComponents3d() {
        updateComponents(components3d);
        
        updateComponents(desiredComponents3d);
    }
    
    private void updateComponents(Pose3d[] components) {
        components[LoggingConstants.CLIMB_INDEX] = new Pose3d(
                LoggingConstants.CLIMB_OFFSET,
                new Rotation3d()
        ); // 0 - climb
        components[LoggingConstants.EXTENDER_INDEX] = new Pose3d(
                LoggingConstants.EXTENDER_OFFSET,
                new Rotation3d()
        ); // 1 - extender
        components[LoggingConstants.INTAKE_INDEX] = new Pose3d(
                LoggingConstants.INTAKE_OFFSET,
                new Rotation3d(
                        0,
                        LoggingConstants.INTAKE_ROTATION_OFFSET,
                        0
                )
        ); // 2 - intake
        components[LoggingConstants.TURRET_INDEX] = new Pose3d(
                LoggingConstants.TURRET_OFFSET,
                new Rotation3d()
        ); // 3 - turret
        components[LoggingConstants.HOOD_INDEX] = new Pose3d(
                LoggingConstants.HOOD_OFFSET,
                new Rotation3d()
        ).rotateAround(
                components[LoggingConstants.TURRET_INDEX].getTranslation(),
                components[LoggingConstants.TURRET_INDEX].getRotation()
        ); // 4 - hood
    }
}

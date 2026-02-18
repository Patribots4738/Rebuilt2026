package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;
import frc.robot.subsystems.superstructure.Superstructure.HopperState;
import frc.robot.subsystems.superstructure.Superstructure.IntakeState;
import frc.robot.subsystems.superstructure.Superstructure.ClimbState;
import frc.robot.subsystems.superstructure.Superstructure.IntakeState;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.ClimbIOKraken;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.ExtenderIOKraken;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIOKraken;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.hood.HoodIOKraken;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.shooter.turret.TurretIOKraken;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.spindexer.SpindexerIOKraken;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.kicker.KickerIOKraken;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.Alignment;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.ShotCalculations;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.custom.PatriBoxController;

public class RobotContainer {

    private PowerDistribution pdh;

    @AutoLogOutput (key = "PowerDistributionHub/TotalCurrent")
    private double totalCurrent;
    @AutoLogOutput (key = "PowerDistributionHub/TotalPower")
    private double totalPower;
    @AutoLogOutput (key = "PowerDistributionHub/Voltage")
    private double voltage;

    private EventLoop testButtonBindingLoop = new EventLoop();
    
    @AutoLogOutput (key = "Draggables/Components3d")
    public static Pose3d[] components3d = {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};
    @AutoLogOutput (key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;
    @SuppressWarnings("unused")
    private final Climb climb;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Intake intake;
    private final Extender extender;
    private final Kicker kicker;
    private final Spindexer spindexer;
    private final Alignment alignment;

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

    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        totalCurrent = pdh.getTotalCurrent();
        SmartDashboard.putNumber("Total Current", totalCurrent);
        totalPower = pdh.getTotalPower();
        SmartDashboard.putNumber("Total Power", totalPower);
        voltage = pdh.getVoltage();
        SmartDashboard.putNumber("Voltage", voltage);

        swerve = new Swerve();
        climb = new Climb(new ClimbIOKraken());
        extender = new Extender(new ExtenderIOKraken());
        intake = new Intake(new IntakeIOKraken());
        spindexer = new Spindexer(new SpindexerIOKraken());
        kicker = new Kicker(new KickerIOKraken());
        flywheel = new Flywheel(new FlywheelIOKraken());
        turret = new Turret(new TurretIOKraken());
        hood = new Hood(new HoodIOKraken());
        shooter = new Shooter(swerve::getPose, swerve::getRobotRelativeVelocity, turret, flywheel, hood);
        superstructure = new Superstructure(extender, intake, climb, shooter, spindexer, kicker);
        alignment = new Alignment(swerve);

        SmartDashboard.putData(field2d);

        driver.back().toggleOnTrue(
                Commands.runOnce(() -> fieldRelativeToggle = !fieldRelativeToggle)
        );
        robotRelativeSupplier = () -> fieldRelativeToggle;

        swerve.setDefaultCommand(new Drive(
                swerve,
                driver::getLeftY,
                driver::getLeftX,
                () -> -driver.getRightX() / 1.6,
                robotRelativeSupplier,
                () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        configureButtonBindings();
        configureTimedEvents();

        pathPlannerStorage = new PathPlannerStorage(Set.of(swerve, extender, intake, climb, shooter, spindexer, kicker));

        prepareNamedCommands();
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
    }

    private void configureButtonBindings() {
        switch(OIConstants.DRIVER_MODE) {
                case DEV:
                        configureDevBindings(driver);
                        break;
                case SINGLE:
                        configureDriverBindings(driver);
                        break;
        }
    }

    private void configureTimedEvents() {}

    private void configureDevBindings(PatriBoxController controller) {
        controller.leftBumper().onTrue(swerve.setWheelsOCommand());
        controller.rightBumper().onTrue(swerve.getSetWheelsX());
        controller.povUp().onTrue(swerve.setWheelsZeroCommand());
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

        controller.leftBumper().onTrue(superstructure.setIntakeState(IntakeState.INTAKE));
        controller.leftTrigger().onTrue(superstructure.setIntakeState(IntakeState.STOW));
        controller.rightBumper().onTrue(superstructure.fullStowCommand());
        controller.rightTrigger()
                .whileTrue(
                        new ActiveConditionalCommand(
                                superstructure.fireCommand(), 
                                Commands.sequence(
                                        shooter.armShooterCommand(), 
                                        Commands.waitUntil(() -> shooter.isReady()),
                                        superstructure.fireCommand()),
                                () -> shooter.isReady()
                        )
                );
        controller.rightTrigger().toggleOnFalse(superstructure.ceaseFireCommand());
        controller.rightStick().onTrue(shooter.setPassIndexCommand(1));
        controller.leftStick().onTrue(shooter.setPassIndexCommand(0));
        controller.povUp().onTrue(superstructure.setClimbState(ClimbState.READY));
        controller.povDown().onTrue(superstructure.setClimbState(ClimbState.STOW).onlyIf(()-> !controller.povRight().getAsBoolean()));
        controller.povDown().and(controller.povRight()).onTrue(superstructure.setClimbState(ClimbState.UP));
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

    public void refreshPDHData() {
        totalCurrent = pdh.getTotalCurrent();
        totalPower = pdh.getTotalPower();
        voltage = pdh.getVoltage();
    }

    public void onDisabled() {
        swerve.stopDriving();
        CommandScheduler.getInstance().schedule(pathPlannerStorage.updatePathViewerCommand());
        pathPlannerStorage.configureAutoChooser();
    }

    public void onEnabled() {
        gameModeStart = Robot.currentTimestamp;
        CommandScheduler.getInstance().schedule(pathPlannerStorage.updatePathViewerCommand());
        freshCode = false;
    }

    private void prepareNamedCommands() {
        NamedCommands.registerCommand("Shoot", shooter.armShooterCommand());
        NamedCommands.registerCommand("ShootUntilEmptyHopper", shooter.armShooterCommand().until(() -> kicker.emptyHopper()));
        NamedCommands.registerCommand("StopShooting", shooter.disarmShooterCommand());
        NamedCommands.registerCommand("Track", shooter.trackTargetCommand(true, true, false));
        NamedCommands.registerCommand("ClimbUp", superstructure.setClimbState(ClimbState.READY));
        NamedCommands.registerCommand("ClimbDown", superstructure.setClimbState(ClimbState.UP));
        NamedCommands.registerCommand("Intake", superstructure.setIntakeState(IntakeState.INTAKE));
        NamedCommands.registerCommand("StopIntake", superstructure.setIntakeState(IntakeState.NO_SPIN));
        NamedCommands.registerCommand("IntakeStow", superstructure.setIntakeState(IntakeState.STOW));

        NamedCommands.registerCommand("TowerAlignLeft", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), -1));
        NamedCommands.registerCommand("TowerAlignRight", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), 1));
        NamedCommands.registerCommand("TowerAlignCenter", alignment.autonoumusTowerAlignmentCommand(swerve.getPose(), 0));

        NamedCommands.registerCommand("WaitUntilEmptyHopper", Commands.waitUntil(() -> kicker.emptyHopper()));
        NamedCommands.registerCommand("WaitUntilBehindTower", Commands.waitUntil(() -> PoseCalculations.behindTower(swerve.getPose())));
        NamedCommands.registerCommand("WaitUntilNotBehindTower", Commands.waitUntil(() -> ! (PoseCalculations.behindTower(swerve.getPose()))));
        NamedCommands.registerCommand("WaitUntilClimb", Commands.waitUntil(() -> superstructure.targetClimbState == ClimbState.UP));
        NamedCommands.registerCommand("WaitUntilInsideAllianceZone", Commands.waitUntil(() -> PoseCalculations.getZone(swerve.getPose()) == PoseCalculations.ZONE.inAllianceZone));
        NamedCommands.registerCommand("WaitUntilOutsideAllianceZone", Commands.waitUntil(() -> PoseCalculations.getZone(swerve.getPose()) != PoseCalculations.ZONE.inAllianceZone));


        NamedCommands.registerCommand("ClimbOrNoClimbThroughDepot", pathPlannerStorage.endAuto("ThroughDepot"));


        NamedCommands.registerCommand("WiggleShoot", Commands.parallel(
                Commands.sequence(Commands.waitSeconds(2.0),
                        NamedCommands.getCommand("IntakeStow")),
                Commands.race(
                        NamedCommands.getCommand("WaitUntilEmptyHopper"),
                        Commands.waitSeconds(5)
                )
        ));
    }
}

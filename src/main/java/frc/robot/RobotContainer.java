package frc.robot;

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
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.ClimbIOKraken;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.ExtenderIOKraken;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIOKraken;
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
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.calc.ShotCalculations;
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
    private final Intake intake;
    private final Extender extender;
    private final Kicker kicker;
    private final Spindexer spindexer;

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
        flywheel = new Flywheel(new FlywheelIOKraken(), swerve);
        turret = new Turret(new TurretIOKraken(), swerve);
        hood = new Hood(new HoodIOKraken(), swerve);
        extender = new Extender(new ExtenderIOKraken());
        intake = new Intake(new IntakeIOKraken());
        spindexer = new Spindexer(new SpindexerIOKraken());
        kicker = new Kicker(new KickerIOKraken());
        superstructure = new Superstructure(extender, intake, climb, turret, hood, flywheel, spindexer, kicker, swerve::getPose);

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
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
    }

    private void configureTimedEvents() {}

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

        controller.leftBumper().onTrue(Commands.runOnce(() -> superstructure.updatePassIndex(0)));


        // controller.leftBumper().whileTrue(swerve.getSetWheelsX());
        // controller.rightBumper().whileTrue(swerve.getSetWheelsO());
        controller.b().whileTrue(swerve.driveCharacterization());
        controller.y().whileTrue(swerve.getSetWheelsZero());
    }

    private void configureOperatorBindings(PatriBoxController controller) {

        // controller.povUp()
        //     .whileTrue(krakenTest.setPosition(() -> krakenTest.getPosition() + 0.2));
        // controller.povDown()
        //     .whileTrue(krakenTest.setPosition(() -> krakenTest.getPosition() - 0.2));
        // controller.a()
        //     .onTrue(krakenTest.setPosition(() -> 0));
        // controller.b()
        //     .whileTrue(krakenTest.setVelocity(() -> 500)
        //         .finallyDo(() -> krakenTest.setVelocity(() -> 0)));
        // controller.x()
        //     .whileTrue(krakenTest.setVelocity(() -> -500)
        //         .finallyDo(() -> krakenTest.setVelocity(() -> 0)));
        // controller.rightTrigger()
        //     .whileTrue(krakenTest.setPercent(controller::getRightY)
        //         .finallyDo(() -> krakenTest.setPercent(() -> 0)));
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

    private void prepareNamedCommands() {}
}

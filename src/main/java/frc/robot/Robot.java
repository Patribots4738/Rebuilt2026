package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.util.Constants.KrakenMotorConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.hardware.phoenix.Kraken;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.system.Timer;

import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

    private static Optional<Alliance> alliance = Optional.empty();
    public static GameMode gameMode = GameMode.DISABLED;
    public static enum GameMode {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    };

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;
    public static boolean exitAuto = false;
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public Robot() {
        // disable CTRE signal autologging 
        SignalLogger.enableAutoLogging(false);

        // Git metadata for tracking version for AKit
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (LoggingConstants.getMode()) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/logs")); // Log to a USB stick
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                break;
            case REPLAY:
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil
                    .addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                    break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                break;
        }
        
        Logger.start(); 

        robotContainer = new RobotContainer();
    }
    
    @Override
    public void driverStationConnected() {
        DriverStationBackend.silenceJoystickConnectionWarning(true);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Used for items like diagnostics
     * ran during disabled, autonomous, teleoperated and test. :D
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Robot.previousTimestamp = Robot.currentTimestamp;
        Robot.currentTimestamp = Timer.getTimestamp();
        CommandScheduler.getInstance().run();
        Robot.alliance = DriverStationBackend.getAlliance();
    }

    @Override
    public void disabledInit() {
        Robot.gameMode = GameMode.DISABLED;
        robotContainer.onDisabled();
    }
    
    @Override
    public void disabledPeriodic() {
        // Now while this may not necessarily be a constant...
        // it needs to be updated.
        DriverStationBackend.refreshData();
        Robot.alliance = DriverStationBackend.getAlliance();
    }

    @Override
    public void disabledExit() {
        RobotContainer.gameModeStart = currentTimestamp;
    }

    @Override   
    public void autonomousInit() {
        Robot.gameMode = GameMode.AUTONOMOUS;
        robotContainer.onEnabled();
        // We only need to update alliance becuase
        // sim GUI starts the bot in a "disconnected"
        // state which won't update the alliance before
        // we enable...
        DriverStationBackend.refreshData();
        Robot.alliance = DriverStationBackend.getAlliance();

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        // Stop our autonomous command if it is still running.
        System.out.printf(
            "*** Auto finished in %.2f secs ***%n", Robot.currentTimestamp - RobotContainer.gameModeStart);
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.settleRobot();
        exitAuto = true;
    }

    @Override
    public void teleopInit() {
        Robot.gameMode = GameMode.TELEOP;
        robotContainer.onEnabled();
        robotContainer.settleRobot();
    }

    @Override
    public void teleopPeriodic() {
    }
	
    @Override
	public void utilityInit() {
        // Cancels all running commands at the start of test mode.
        Robot.gameMode = GameMode.TEST;
        CommandScheduler.getInstance().cancelAll();
        robotContainer.onEnabled();
    }
    
    @Override
    public void utilityPeriodic() {
    }

    @Override
    public void utilityExit() {
        // Switch back to the normal button loop!
        CommandScheduler.getInstance().setActiveButtonLoop(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
	
	@Override
    public void simulationPeriodic() {
        for (Kraken kraken : KrakenMotorConstants.KRAKEN_MOTOR_MAP.values()) {
            kraken.tick();
        }   
    }

    public static boolean isRedAlliance() {
        return alliance.equals(Optional.of(Alliance.RED));
    }

    public static boolean isBlueAlliance() {
        return alliance.equals(Optional.of(Alliance.BLUE));
    }
}

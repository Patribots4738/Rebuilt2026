package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Swerve;

import static edu.wpi.first.units.Units.*;

public class SysIdSwerve {
	private final SysIdRoutine driveRoutine;
	private final SysIdRoutine turnRoutine;
	
	public SysIdSwerve(Swerve swerve) {
		driveRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> swerve.runDriveCharacterization(volts.in(Volts)),
						log -> log.motor("drive")
							.voltage(Volts.of(swerve.getDriveCharacterizationVoltage()))
							.angularVelocity(RadiansPerSecond.of(swerve.getDriveCharacterizationVelocity())),
						swerve
				)
		);
		
		turnRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> swerve.runTurnCharacterization(volts.in(Volts)),
						log -> log.motor("turn")
							.voltage(Volts.of(swerve.getTurnCharacterizationVoltage()))
							.angularVelocity(RadiansPerSecond.of(swerve.getTurnCharacterizationVelocity())),
						swerve
				)
		);
	}
	
	public Command quasistaticDrive(SysIdRoutine.Direction direction) {
		return driveRoutine.quasistatic(direction);
	}
	
	public Command dynamicDrive(SysIdRoutine.Direction direction) {
		return driveRoutine.dynamic(direction);
	}
	
	public Command quasistaticTurn(SysIdRoutine.Direction direction) {
		return turnRoutine.quasistatic(direction);
	}
	
	public Command dynamicTurn(SysIdRoutine.Direction direction) {
		return turnRoutine.dynamic(direction);
	}
}

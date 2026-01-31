package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;

import static edu.wpi.first.units.Units.*;

public class SysIdFlywheel {
	private final SysIdRoutine flywheelRoutine;
	
	public SysIdFlywheel(Flywheel flywheel) {
		flywheelRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> flywheel.runCharacterization(volts.in(Volts)),
						log -> log.motor("drive")
							.voltage(Volts.of(flywheel.getVoltage()))
							.linearVelocity(MetersPerSecond.of(flywheel.getVelocity())),
						flywheel
				)
		);
	}
	
	public Command quasistaticFlywheelCommand(SysIdRoutine.Direction direction) {
		return flywheelRoutine.quasistatic(direction);
	}
	
	public Command dynamicFlywheelCommand(SysIdRoutine.Direction direction) {
		return flywheelRoutine.dynamic(direction);
	}
}

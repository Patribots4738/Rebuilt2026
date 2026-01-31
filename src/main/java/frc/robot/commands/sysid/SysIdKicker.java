package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.kicker.Kicker;

import static edu.wpi.first.units.Units.*;

public class SysIdKicker {
	private final SysIdRoutine kickerRoutine;
	
	public SysIdKicker(Kicker kicker) {
		kickerRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> kicker.runCharacterization(volts.in(Volts)),
						log -> log.motor("kicker")
							.voltage(Volts.of(kicker.getVoltage()))
							.linearVelocity(MetersPerSecond.of(kicker.getCharacterizationVelocity())),
						kicker
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return kickerRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return kickerRoutine.dynamic(direction);
	}
}

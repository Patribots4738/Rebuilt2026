package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.extender.Extender;

import static edu.wpi.first.units.Units.*;

public class SysIdExtender {
	private final SysIdRoutine extenderRoutine;
	
	public SysIdExtender(Extender extender) {
		extenderRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> extender.runCharacterization(volts.in(Volts)),
						log -> log.motor("extender")
							.voltage(Volts.of(extender.getVoltage()))
							.linearVelocity(MetersPerSecond.of(extender.getCharacterizationVelocity())),
						extender
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return extenderRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return extenderRoutine.dynamic(direction);
	}
}

package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;

import static edu.wpi.first.units.Units.*;

public class SysIdSpindexer {
	private final SysIdRoutine spindexerRoutine;
	
	public SysIdSpindexer(Spindexer spindexer) {
		spindexerRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> spindexer.setVoltage(volts.in(Volts)),
						log -> log.motor("spindexer")
							.voltage(Volts.of(spindexer.getVoltage()))
							.linearVelocity(MetersPerSecond.of(spindexer.getVelocity())),
						spindexer
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return spindexerRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return spindexerRoutine.dynamic(direction);
	}
}

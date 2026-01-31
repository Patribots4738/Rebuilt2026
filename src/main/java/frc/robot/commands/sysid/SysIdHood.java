package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.shooter.hood.Hood;

import static edu.wpi.first.units.Units.*;

public class SysIdHood {
	private final SysIdRoutine hoodRoutine;
	
	public SysIdHood(Hood hood) {
		hoodRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> hood.runCharacterization(volts.in(Volts)),
						log -> log.motor("hood")
							.voltage(Volts.of(hood.getVoltage()))
							.linearVelocity(MetersPerSecond.of(hood.getVelocity())),
						hood
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return hoodRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return hoodRoutine.dynamic(direction);
	}
}

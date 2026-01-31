package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.shooter.flywheel.Flywheel;

import static edu.wpi.first.units.Units.*;

public class SysIdClimb {
	private final SysIdRoutine climbRoutine;
	
	public SysIdClimb(Climb climb) {
		climbRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> climb.runCharacterization(volts.in(Volts)),
						log -> log.motor("climb")
							.voltage(Volts.of(climb.getVoltage()))
							.linearVelocity(MetersPerSecond.of(climb.getCharacterizationVelocity())),
						climb
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return climbRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return climbRoutine.dynamic(direction);
	}
}

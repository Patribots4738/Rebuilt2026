package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.shooter.turret.Turret;

import static edu.wpi.first.units.Units.*;

public class SysIdTurret {
	private final SysIdRoutine turretRoutine;
	
	public SysIdTurret(Turret turret) {
		turretRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> turret.runCharacterization(volts.in(Volts)),
						log -> log.motor("turret")
							.voltage(Volts.of(turret.getVoltage()))
							.linearVelocity(MetersPerSecond.of(turret.getVelocity())),
						turret
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return turretRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return turretRoutine.dynamic(direction);
	}
}

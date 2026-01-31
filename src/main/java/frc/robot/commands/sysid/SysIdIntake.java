package frc.robot.commands.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.intake.Intake;

import static edu.wpi.first.units.Units.*;

public class SysIdIntake {
	private final SysIdRoutine intakeRoutine;
	
	public SysIdIntake(Intake intake) {
		intakeRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.per(Seconds).of(1),
						Volts.of(7),
						Seconds.of(10)
				),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> intake.runCharacterization(volts.in(Volts)),
						log -> log.motor("intake")
							.voltage(Volts.of(intake.getVoltage()))
							.linearVelocity(MetersPerSecond.of(intake.getCharacterizationVelocity())),
						intake
				)
		);
	}
	
	public Command quasistatic(SysIdRoutine.Direction direction) {
		return intakeRoutine.quasistatic(direction);
	}
	
	public Command dynamic(SysIdRoutine.Direction direction) {
		return intakeRoutine.dynamic(direction);
	}
}

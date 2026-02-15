// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.LoggingConstants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double percentOutput = 0.0;
    @AutoLogOutput (key = "Subsystems/Intake/ShouldRunSetpoint")
    private boolean shouldRunSetpoint = false;
    private double testPercent;

    public Intake(IntakeIO io) {
        this.io = io;

        IntakeConstants.TEST_PERCENT.onChanged(Commands.runOnce(() -> testPercent = IntakeConstants.TEST_PERCENT.get()).ignoringDisable(true));
        // fastVelocity.onChanged().or(fastAcceleration.onChanged()).onTrue(Commands.runOnce(() -> fastProfile = new TrapezoidProfile(new Constraints(fastVelocity.get(), fastAcceleration.get()))).ignoringDisable(true));
        // slowVelocity.onChanged().or(slowAcceleration.onChanged()).onTrue(Commands.runOnce(() -> slowProfile = new TrapezoidProfile(new Constraints(slowVelocity.get(), slowAcceleration.get()))).ignoringDisable(true));

        // IntakeConstants.LOGGED_GAINS.onChanged(Commands.runOnce(() -> io.setGains(IntakeConstants.LOGGED_GAINS.get())).ignoringDisable(true));
        // brakeMotor.onChanged(Commands.runOnce(() -> io.setBrakeMode(brakeMotor.get())).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (shouldRunSetpoint) {
            io.setPercent(percentOutput);
        } else {
            io.setNeutral();
        }

    }

    public void setPercent(double percent) {
        percentOutput = percent;
        shouldRunSetpoint = true;
    }

    public void setNeutral() {
        shouldRunSetpoint = false;
    }

    public void setTestPercent() {
        setPercent(testPercent);
    }

    public Command setTestPercentCommand() {
        return Commands.runOnce(() -> this.setTestPercent());
    }

    public Command setPercentCommand(DoubleSupplier percentSupplier) {
        return runOnce(() -> setPercent(percentSupplier.getAsDouble()));
    }

    public Command setPercentCommand(double percent) {
        return setPercentCommand(() -> percent);
    }

    public Command setNeutralCommand() {
        return runOnce(this::setNeutral);
    }
}
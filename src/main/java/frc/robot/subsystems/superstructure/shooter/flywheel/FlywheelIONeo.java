// package frc.robot.subsystems.superstructure.shooter.flywheel;

// import frc.robot.util.Constants.FlywheelConstants;
// import frc.robot.util.custom.GainConstants;
// import frc.robot.util.hardware.rev.Neo;

// public class FlywheelIONeo implements FlywheelIO {
//     private final Neo motor;

//     public FlywheelIONeo() {
//         motor = new Neo(FlywheelConstants.CAN_ID, false, false, false);
//         configMotor();
//     }

//     private void configMotor() {
//         motor.setPID(FlywheelConstants.GAINS);
//         motor.setInverted(FlywheelConstants.MOTOR_INVERTED);
//         motor.setSmartCurrentLimit((int) FlywheelConstants.CURRENT_LIMIT);
//     }

//     @Override
//     public void updateInputs(FlywheelIOInputs inputs) {
//         inputs.motorConnected = true;
//         inputs.speedRPM = motor.getVelocity();
//         inputs.targetSpeedRPM = motor.getTargetVelocity();
//         inputs.appliedOutputVolts = motor.getAppliedOutput() * motor.getBusVoltage();
//         inputs.torqueCurrentAmps = motor.getOutputCurrent();
//         inputs.temperatureCelsius = motor.getMotorTemperature();
//     }

//     @Override
//     public void runCharacterization(double input) {
//         motor.setVoltage(input);
//     }

//     @Override
//     public void setBrakeMode(boolean brake) {
//         if (brake) {
//             motor.setBrakeMode();
//         }
//         else {
//             motor.setCoastMode();
//         }
//     }

//     @Override
//     public void setSpeedRPM(double speed) {
//         motor.setTargetVelocity(speed);
//     }

//     @Override
//     public void setPercent(double percent) {
//         motor.setTargetPercent(percent);
//     }

//     @Override
//     public void setGains(GainConstants constants) {
//         motor.setPID(constants);
//     }
// }

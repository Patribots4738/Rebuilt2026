// package frc.robot.subsystems.superstructure.intake;

// import frc.robot.util.Constants.IntakeConstants;
// import frc.robot.util.hardware.rev.Neo;

// public class IntakeIONeo implements IntakeIO {

//     private final Neo motor;

//     public IntakeIONeo() {
//         motor = new Neo(IntakeConstants.CAN_ID, false);
//         configMotor();
//     }

//     private void configMotor() {
//         motor.setInverted(IntakeConstants.MOTOR_INVERTED);
//         motor.setSmartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT);
//         setBrakeMode(IntakeConstants.BRAKE_MOTOR);
//     }

//     @Override
//     public void updateInputs(IntakeIOInputs inputs) {
//         inputs.motorConnected = true;
//         inputs.percentOutput = motor.getAppliedOutput();
//         inputs.targetPercentOutput = motor.getTargetPercent();
//         inputs.appliedOutputVolts = motor.getAppliedOutput() * motor.getBusVoltage();
//         inputs.torqueCurrentAmps = motor.getOutputCurrent();
//         inputs.temperatureCelsius = motor.getMotorTemperature();
//         inputs.positionRotations = motor.getPosition();
//         inputs.velocityRotationsPerSec = motor.getVelocity() / 60;
//     }

//     @Override
//     public void setBrakeMode(boolean brake) {
//         if (brake) {
//             motor.setBrakeMode();

//         } else {
//             motor.setCoastMode();

//         }
//     }
    
//     @Override
//     public void setPercent(double percent) {
//         motor.setTargetPercent(percent);
//     }
// }
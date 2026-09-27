package frc.robot.subsystems.superstructure.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.util.Constants;
import frc.robot.util.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
    private DCMotor motorModel;
    private FlywheelSim simPhysics;

    // get motor model
    public FlywheelIOSim() {
        if (FlywheelConstants.USE_FOC == true) {
            motorModel = FlywheelConstants.MOTOR_TYPE == 44 ? DCMotor.getKrakenX44Foc : DCMotor.getKrakenX60Foc;
        }
        else {
            motorModel = FlywheelConstants.MOTOR_TYPE == 44 ? DCMotor.getKrakenX44 : DCMotor.getKrakenX60;
        }

        // create physics simulation
        sim = new FlywheelIOSim(
            LinearSystemId.createFlywheelSystem(
                motorModel,
                FlywheelConstants.MASS_KG,
                FlywheelConstants.RADIUS_METERS,
                FlywheelConstants.GEAR_RATIO
            )
        );
    }
}

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import frc.robot.utilities.REVUtility;

public class GyroIOSimulation implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSimulation(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadiansPerSecond = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

        inputs.odometryYawTimestamps = REVUtility.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}

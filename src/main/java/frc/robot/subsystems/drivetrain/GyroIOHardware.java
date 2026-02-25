package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants;

public class GyroIOHardware implements GyroIO {
    private final AHRS gyro;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOHardware() {
        gyro = new AHRS(NavXComType.kMXP_SPI, (byte) DrivetrainConstants.kOdometryFrequency);

        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();

        inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngle() * -1);
        inputs.pitchPosition = Rotation2d.fromDegrees(gyro.getPitch() * -1);
        inputs.rollPosition = Rotation2d.fromDegrees(gyro.getRoll() * -1);

        inputs.yawVelocityRadiansPerSecond = Units.degreesToRadians(gyro.getRawGyroZ() * -1);
        inputs.pitchVelocityRadiansPerSecond = Units.degreesToRadians(gyro.getRawGyroX() * -1);
        inputs.rollVelocityRadiansPerSecond = Units.degreesToRadians(gyro.getRawGyroY() * -1);

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map((Double v) -> Rotation2d.fromDegrees(v * -1)).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}

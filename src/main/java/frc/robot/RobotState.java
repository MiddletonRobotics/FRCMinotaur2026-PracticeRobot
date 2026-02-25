package frc.robot;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.math.ConcurrentTimeInterpolatableBuffer;
import frc.robot.constants.GlobalConstants;

public class RobotState {
    private final Consumer<WeightedPoseEstimate> visionEstimateConsumer;

    public RobotState(Consumer<WeightedPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        fieldToRobot.addSample(0.0, Pose2d.kZero);
    }

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer.createBuffer(GlobalConstants.kLoopBackTimeSeconds);

    private double lastUsedPoseTimestamp = 0;
    private Pose2d lastUsedPoseEstimate = Pose2d.kZero;

    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawPositionRadians = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchPositionRadians = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollPositionRadians = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelX = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelY = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void addDriveMotionMeasurements(
        double timestamp,
        double yawAngularVelocity,
        double pitchAngularVelocity,
        double rollAngularVelocity,
        double yawAngularPosition,
        double pitchAngularPosition,
        double rollAngularPosition,
        double accelerationX,
        double accelerationY,
        ChassisSpeeds desiredRobotRelativeChassisSpeeds,
        ChassisSpeeds desiredFieldRelativeChassisSpeeds,
        ChassisSpeeds measuredRobotRelativeChassisSpeeds,
        ChassisSpeeds measuredFieldRelativeChassisSpeeds
    ) {
        driveYawAngularVelocity.addSample(timestamp, yawAngularVelocity);
        drivePitchAngularVelocity.addSample(timestamp, pitchAngularVelocity);
        driveRollAngularVelocity.addSample(timestamp, rollAngularVelocity);
        driveYawPositionRadians.addSample(timestamp, yawAngularPosition);
        drivePitchPositionRadians.addSample(timestamp, pitchAngularPosition);
        driveRollPositionRadians.addSample(timestamp, rollAngularPosition);

        driveAccelX.addSample(timestamp, accelerationX);
        driveAccelY.addSample(timestamp, accelerationY);

        this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeChassisSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredRobotRelativeChassisSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeChassisSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? Pose2d.kZero : maybeFieldToRobot.getValue();

        var delta = getLatestMeasuredRobotRelativeChassisSpeeds();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot.exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredRobotRelativeChassisSpeeds() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeeds() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestMeasuredRobotRelativeChassisSpeeds();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;

        return speeds;
    }

    private Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);

        if (max.isEmpty() || min.isEmpty()) {
            return Optional.empty();
        }

        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if (Robot.isReal()) {
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        }

        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(
            double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updatePoseEstimate(WeightedPoseEstimate poseEstimate) {
        lastUsedPoseTimestamp = poseEstimate.getTimestampSeconds();
        lastUsedPoseEstimate = poseEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(poseEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedPoseTimestamp;
    }

    public Pose2d lastUsedMegatagPose() {
        return lastUsedPoseEstimate;
    }
}

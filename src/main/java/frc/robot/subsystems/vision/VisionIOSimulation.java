package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.minolib.vision.CameraConfiguration;
import frc.robot.constants.VisionConstants;

public class VisionIOSimulation extends VisionIOPhotonVision {
  private static final double DIAGONAL_FOV = 96.0; 
  private static final int kImgWidth = 1600; 
  private static final int kImgHeight = 1200; 

  private Supplier<Pose2d> poseSupplier;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  public VisionIOSimulation(final String cameraName, final CameraConfiguration cameraConfiguration, AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier) {
    super(cameraName, cameraConfiguration, layout);

    this.poseSupplier = poseSupplier;

    this.visionSim = new VisionSystemSim(cameraName);
    this.visionSim.addAprilTags(layout);
    
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(kImgWidth, kImgHeight, Rotation2d.fromDegrees(DIAGONAL_FOV));
    cameraProp.setCalibError(VisionConstants.kSimAverageErrorPixels, VisionConstants.kSimErrorStdDevPixels);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(100);
    cameraProp.setLatencyStdDevMs(30);

    this.cameraSim = new PhotonCameraSim(camera, cameraProp, layout);
    visionSim.addCamera(cameraSim, cameraConfiguration.getTransformOffset());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    this.visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.CameraConfiguration.CameraLocation;

public class VisionConstants {
      public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      public static final String kFrontLeftCameraName = "frontLeft";
      public static final String kFrontRightCameraName = "frontRight";

      public static final CameraConfiguration kFrontLeftConfiguration = new CameraConfiguration()
            .withCameraName(kFrontLeftCameraName)
            .withCameraLocation(CameraLocation.FRONT_LEFT)
            .withLengthOffset(Inches.of(9.2837556))
            .withWidthOffset(Inches.of(1.6423085))
            .withHeightOffset(Inches.of(6.9584678))
            .withMountingRoll(Degrees.of(340))
            .withMountingYaw(Degrees.of(324));

      public static final CameraConfiguration kFrontRightConfiguration = new CameraConfiguration()
            .withCameraName(kFrontLeftCameraName)
            .withCameraLocation(CameraLocation.FRONT_LEFT)
            .withLengthOffset(Inches.of(9.2837556))
            .withWidthOffset(Inches.of(1.6423085))
            .withHeightOffset(Inches.of(6.9584678))
            .withMountingRoll(Degrees.of(340))
            .withMountingYaw(Degrees.of(324));

      public static double kMaximumTagAmbiguity = 0.5;
      public static Distance kMaximumZPoseError = Inches.of(0.625);

      public static double linearStdDevBaseline = 0.02; // Meters
      public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

      public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
      };

      public static double linearStdDevMegatag2Factor = 0.5; 
      public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; 

      public static final double kSimAverageErrorPixels = 0.1;
      public static final double kSimErrorStdDevPixels = 0.05;
}

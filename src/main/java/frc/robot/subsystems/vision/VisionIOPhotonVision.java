package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** The IO layer for one SecondSight camera */
public class VisionIOPhotonVision implements VisionIO {
  final PhotonCamera cam = new PhotonCamera("Arducam_OV2311_USB_Camera (1)");
  final Transform3d robotToCam =
      new Transform3d(
          new Translation3d(2 * 0.0254, 0 * 0.0254, 24 * 0.0254),
          new Rotation3d(0, -25.0 * Math.PI / 180, -4.6 * Math.PI / 180));

  // Construct PhotonPoseEstimator
  final PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), // TODO: Fix
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCam);

  /**
   * Constructs a new <code>VisionIOPhotonVision</code> from a NT path
   *
   * @param inst_name The NT path of the instance
   * @param cam_name The name of the camera
   */
  public VisionIOPhotonVision(String inst_name, String cam_name) {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var urres = cam.getAllUnreadResults();
    if (!urres.isEmpty()) {
      var k = photonPoseEstimator.update(urres.get(0));
      inputs.pose = Optional.ofNullable(k.get().estimatedPose);
      inputs.timestamp = Optional.of(k.get().timestampSeconds);

    } else {
      inputs.pose = Optional.empty();
      inputs.timestamp = Optional.empty();
    }
  }
}

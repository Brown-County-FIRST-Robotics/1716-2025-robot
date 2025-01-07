package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.utils.*;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class Vision extends PeriodicRunnable {
  final Transform3d[] camPoses;
  final VisionIO[] ios;
  VisionIOInputs[] inputs;
  Drivetrain drivetrain;
  AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  LoggedTunableNumber oneTagTranslationStdDev =
      new LoggedTunableNumber("Vision/One tag Translation StdDev", 1);
  LoggedTunableNumber oneTagRotationStdDev =
      new LoggedTunableNumber("Vision/One tag Rotation StdDev", 1);
  LoggedTunableNumber multiTagTranslationStdDev =
      new LoggedTunableNumber("Vision/Multi tag Translation StdDev", 0.05);
  LoggedTunableNumber multiTagRotationStdDev =
      new LoggedTunableNumber("Vision/Multi tag Rotation StdDev", 1);
  LoggedTunableNumber maxRMSError = new LoggedTunableNumber("Vision/Max RMS Error", 0.85);
  final CustomAlerts.TimeoutAlert visionWatchDog =
      new CustomAlerts.TimeoutAlert(Alert.AlertType.WARNING, 10, "Vision timeout");
  OverridePanel overridePanel;

  /**
   * Constructs a <code>Vision</code> subsystem
   *
   * @param drivetrain The drivetrain to send the updates to
   * @param camPoses The positions of the cameras
   * @param ios The IOs of the cameras
   */
  public Vision(
      Drivetrain drivetrain, Transform3d[] camPoses, VisionIO[] ios, OverridePanel overridePanel_) {
    super();
    this.camPoses = camPoses;
    this.ios = ios;
    this.overridePanel = overridePanel_;
    if (camPoses.length != ios.length) {
      throw new IllegalArgumentException("Number of IOs and camera poses do not match");
    }
    this.inputs = new VisionIOInputs[ios.length];
    for (int i = 0; i < ios.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Inputs_" + i, inputs[i]);
      if (inputs[i].pose.isPresent() && inputs[i].timestamp.isPresent()) {
        visionWatchDog.feed();
        Pose3d outPose = inputs[i].pose.get();
        Logger.recordOutput("Vision/EstPose_" + i, outPose);
        if (!overridePanel.disableVision().getAsBoolean()
            && (ShootWhileMove.getFieldRelativeSpeeds(drivetrain.getVelocity(), new Rotation2d())
                    .getNorm()
                < 0.8)
            && Math.abs(drivetrain.getVelocity().omegaRadiansPerSecond) < 0.5) {
          drivetrain.addVisionUpdate(
              outPose.toPose2d(), VecBuilder.fill(0.1, 0.1, 1), inputs[i].timestamp.get());
        }
      }
    }
  }
}

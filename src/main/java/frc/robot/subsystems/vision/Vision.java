package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.*;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class Vision extends PeriodicRunnable {
  //  final Transform3d[] camPoses;
  //  final VisionIO[] ios;
  //  VisionIOInputs[] inputs; //TEMP:add back in
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

  VisionSLAMIO slamio;
  VisionSLAMIOInputsAutoLogged slamInputs = new VisionSLAMIOInputsAutoLogged();
  Transform3d slamPose;
  Pose3d zero = Pose3d.kZero;
  Pose3d lastSee = Pose3d.kZero;
  boolean seen = false;

  public void setpos(Pose2d pos) {
    zero = lastSee;
    zero = zero.plus(new Transform3d(Pose3d.kZero, new Pose3d(pos)).inverse());
  }

  public Vision(Drivetrain drivetrain, Transform3d slamPose, VisionSLAMIO slamio) {
    this.drivetrain = drivetrain;
    this.slamPose = slamPose;
    this.slamio = slamio;
    drivetrain.getPE().pt = this;
  }

  /**
   * Constructs a <code>Vision</code> subsystem
   *
   * @param drivetrain The drivetrain to send the updates to
   * @param camPoses The positions of the cameras
   * @param ios The IOs of the cameras
   */
  //  public Vision(
  //      Drivetrain drivetrain, Transform3d[] camPoses, VisionIO[] ios, OverridePanel
  // overridePanel_) {
  //    super();
  //    this.camPoses = camPoses;
  //    this.ios = ios;
  //    this.overridePanel = overridePanel_;
  //    if (camPoses.length != ios.length) {
  //      throw new IllegalArgumentException("Number of IOs and camera poses do not match");
  //    }
  //    this.inputs = new VisionIOInputs[ios.length];
  //    for (int i = 0; i < ios.length; i++) {
  //      inputs[i] = new VisionIOInputs();
  //    }
  //    this.drivetrain = drivetrain;
  //  } //TEMP: add this

  @Override
  public void periodic() {
    slamio.updateInputs(slamInputs);
    Logger.processInputs("Vision/SLAM", slamInputs);

    if (slamInputs.present) {
      var q = slamInputs.questQuat.toRotationVector();
      var rotation = new Rotation3d(VecBuilder.fill(-q.get(0), -q.get(2), q.get(1)));
      var cpos =
          new Pose3d(
              -slamInputs.questTranslation.getZ(),
              -slamInputs.questTranslation.getX(),
              -slamInputs.questTranslation.getY(),
              rotation);
      cpos = cpos.plus(slamPose.inverse());
      if (new XboxController(0).getAButtonPressed()) {
        seen = false;
      }
      if (!seen) {
        seen = true;
        lastSee = cpos;
        zero = cpos;
      }

      //          p = p.plus(cpos.minus(lp));
      Logger.recordOutput("asdf", Pose3d.kZero.plus(cpos.minus(zero)));

      lastSee = cpos;
    }

    // TEMP: Add this back in
    //    for (int i = 0; i < ios.length; i++) {
    //      ios[i].updateInputs(inputs[i]);
    //      Logger.processInputs("Vision/Inputs_" + i, inputs[i]);
    //      if (inputs[i].pose.isPresent() && inputs[i].timestamp.isPresent()) {
    //        visionWatchDog.feed();
    //        Pose3d outPose = inputs[i].pose.get();
    //        Logger.recordOutput("Vision/EstPose_" + i, outPose);
    //        if (!overridePanel.disableVision().getAsBoolean()
    //            && (ShootWhileMove.getFieldRelativeSpeeds(drivetrain.getVelocity(), new
    // Rotation2d())
    //                    .getNorm()
    //                < 0.8)
    //            && Math.abs(drivetrain.getVelocity().omegaRadiansPerSecond) < 0.5) {
    //          drivetrain.addVisionUpdate(
    //              outPose.toPose2d(), VecBuilder.fill(0.1, 0.1, 1), inputs[i].timestamp.get());
    //        }
    //      }
    //    }
  }
}

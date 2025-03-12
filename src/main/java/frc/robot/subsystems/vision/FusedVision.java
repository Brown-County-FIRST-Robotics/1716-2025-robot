package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.*;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class FusedVision extends PeriodicRunnable {
  final VisionIO io;
  public VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  Drivetrain drivetrain;

  final CustomAlerts.TimeoutAlert visionWatchDog =
      new CustomAlerts.TimeoutAlert(Alert.AlertType.WARNING, 10, "Vision timeout");
  OverridePanel overridePanel;

  VisionSLAMIO slamio;
  VisionSLAMIOInputsAutoLogged slamInputs = new VisionSLAMIOInputsAutoLogged();
  Transform3d slamPose;
  Pose3d zero = Pose3d.kZero;
  Pose3d lastHeadsetPoseSeen = Pose3d.kZero;
  boolean seen = false;

  public void setpos(Pose2d pos) {
    zero = lastHeadsetPoseSeen;
    zero = zero.plus(new Transform3d(Pose3d.kZero, new Pose3d(pos)).inverse());
  }

  public FusedVision(
      Drivetrain drivetrain, Transform3d slamPose, VisionSLAMIO slamio, VisionIO visionIO) {
    this.drivetrain = drivetrain;
    this.slamPose = slamPose;
    this.slamio = slamio;
    drivetrain.getPE().pt = this;
    this.io = visionIO;
    new CustomAlerts.CustomAlert(
        Alert.AlertType.WARNING,
        () -> slamInputs.battPercent < 20.0,
        () -> "Quest battery is at " + Double.toString(slamInputs.battPercent) + "%");
  }

  public Pose2d getSlamPose() {
    return Pose3d.kZero.plus(lastHeadsetPoseSeen.minus(zero)).toPose2d();
  }

  long lastFrame = 0;

  long frameTimeDelta = 0;

  public boolean isActive() {
    return frameTimeDelta > 0;
  }

  @Override
  public void periodic() {
    slamio.updateInputs(slamInputs);
    Logger.processInputs("Vision/SLAM", slamInputs);

    frameTimeDelta = slamInputs.frames - lastFrame;
    lastFrame = slamInputs.frames;
    Logger.recordOutput("Vision/QuestConnected", isActive());

    if (slamInputs.present) {
      var questRotVec = slamInputs.questQuat.toRotationVector(); // In quest coordinate system
      var headsetRotation =
          new Rotation3d(
              VecBuilder.fill(
                  questRotVec.get(0),
                  -questRotVec.get(2),
                  -questRotVec.get(1))); // In WPILIB coordinates
      var headsetPos =
          new Pose3d(
              -slamInputs.questTranslation.getZ(),
              -slamInputs.questTranslation.getX(),
              -slamInputs.questTranslation.getY(),
              headsetRotation);
      var robotHeadsetPos = headsetPos.plus(slamPose.inverse());
      if (new XboxController(0).getAButtonPressed()) {
        seen = false;
      }

      if (!seen) {
        seen = true;
        lastHeadsetPoseSeen = robotHeadsetPos;
        zero = robotHeadsetPos;
      }

      Logger.recordOutput("asdf", Pose3d.kZero.plus(robotHeadsetPos.minus(zero)));

      lastHeadsetPoseSeen = robotHeadsetPos;
    }
    io.updateInputs(inputs);
    Logger.processInputs("Vision/AprilInputs", inputs);
    // TEMP: Add some of this back in
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

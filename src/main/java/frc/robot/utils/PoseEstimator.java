package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.vision.FusedVision;

/** A pose estimator that fuses vision and odometry updates */
public class PoseEstimator {
  // Ship of Theseused from
  public FusedVision pt;
  // https://github.com/wpilibsuite/allwpilib/blob/1db3936965bd8ed33224ad388cf9f16d12999a08/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java
  Pose2d current = Pose2d.kZero;
  boolean usedVis = false;

  public Pose2d getPose() {
    return pt.getSlamPose();
  }

  public void setPose(Pose2d pz) {
    pt.setpos(pz);
  }

  public void feed() {
    if (new XboxController(0).getAButtonPressed()) {
      usedVis = false;
    }
    if (DriverStation.isDisabled() && !usedVis && pt.isActive()) {
      if (pt.inputs.pose.isPresent()) {
        pt.setpos(pt.inputs.pose.get().toPose2d());
        usedVis = true;
      }
    }
  }

  /** Constructs a new pose estimator */
  public PoseEstimator() {
    new PeriodicRunnable() {
      @Override
      public void periodic() {
        feed();
      }
    };
  }
}

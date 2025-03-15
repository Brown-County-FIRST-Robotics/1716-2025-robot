package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.vision.FusedVision;
import java.util.Optional;

/** A pose estimator that fuses vision and odometry updates */
public class PoseEstimator {
  // Ship of Theseused from
  public Optional<FusedVision> pt = Optional.empty();
  // https://github.com/wpilibsuite/allwpilib/blob/1db3936965bd8ed33224ad388cf9f16d12999a08/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java
  Pose2d current = Pose2d.kZero;
  boolean usedVis = false;

  public Pose2d getPose() {
    if (pt.isPresent()) return pt.get().getSlamPose();
    else return Pose2d.kZero;
  }

  public void setPose(Pose2d pz) {
    pt.ifPresent(fusedVision -> fusedVision.setpos(pz));
  }

  public void feed() {
    if (new XboxController(0).getAButtonPressed()) {
      usedVis = false;
    }
    if (pt.isPresent()) {
      if (DriverStation.isDisabled() && !usedVis && pt.get().isActive()) {
        if (pt.get().inputs.pose.isPresent()) {
          pt.get().setpos(pt.get().inputs.pose.get().toPose2d());
          usedVis = true;
        }
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

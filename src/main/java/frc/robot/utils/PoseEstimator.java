package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.Vision;
import java.util.*;

/** A pose estimator that fuses vision and odometry updates */
public class PoseEstimator {
  // Ship of Theseused from
  public Vision pt;
  // https://github.com/wpilibsuite/allwpilib/blob/1db3936965bd8ed33224ad388cf9f16d12999a08/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java
  Pose2d current = Pose2d.kZero;

  public Pose2d getPose() {
    return pt.getPose();
  }

  public void setPose(Pose2d pz) {
    pt.setpos(pz);
  }

  /** Constructs a new pose estimator */
  public PoseEstimator() {}
}

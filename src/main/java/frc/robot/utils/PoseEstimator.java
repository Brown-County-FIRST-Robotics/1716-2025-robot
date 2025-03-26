package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.FusedVision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

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
    if (new XboxController(0).getXButtonPressed()) {
      Pose2d face = FieldConstants.getFace(0);
      Pose2d plus = face.plus(new Transform2d(16.0 * 0.0254, 0, Rotation2d.k180deg));
      Logger.recordOutput("dsffsddfs", face);
      setPose(
          FieldConstants.getFace(0).plus(new Transform2d(-16.0 * 0.0254, 0, Rotation2d.k180deg)));
    }
    if (pt.isPresent()) {
      if (!usedVis && pt.get().isActive()) {
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

package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  class WristIOInputs {
    double angle = 0.0;
    double omega = 0.0;

    double appliedOutput = 0.0;
    double temperature = 0.0;
    double current = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setPosition(Rotation2d commandPosition, double arbFF) {}
}

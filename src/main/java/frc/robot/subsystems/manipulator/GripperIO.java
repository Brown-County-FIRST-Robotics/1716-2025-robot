package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  class GripperIOInputs {
    double topPosition = 0.0;
    double bottomPosition = 0.0;
    double rearPosition = 0.0;
    double topVelocity = 0.0;
    double bottomVelocity = 0.0;
    double rearVelocity = 0.0;

    double topAppliedOutput = 0.0;
    double bottomAppliedOutput = 0.0;
    double rearAppliedOutput = 0.0;
    double topTemperature = 0.0;
    double bottomTemperature = 0.0;
    double rearTemperature = 0.0;
    double topCurrent = 0.0;
    double bottomCurrent = 0.0;
    double rearCurrent = 0.0;
  }

  default void updateInputs(GripperIOInputs inputs) {}

  default void setVelocities(
      double topCommandVelocity, double bottomCommandVelocity, double rearCommandVelocity) {}
}

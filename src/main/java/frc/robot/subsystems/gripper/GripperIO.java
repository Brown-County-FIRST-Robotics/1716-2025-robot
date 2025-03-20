package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  class GripperIOInputs {
    double topPosition = 0.0;
    double bottomPosition = 0.0;
    double topVelocity = 0.0;
    double bottomVelocity = 0.0;
    double coralLaserDistance = 0.0;
    double algaeLaserDistance = 0.0;
    boolean hasCoralLaserMeasurement = false;
    boolean hasAlgaeLaserMeasurement = false; // Whether the laser can see anything

    double topAppliedOutput = 0.0;
    double bottomAppliedOutput = 0.0;
    double topTemperature = 0.0;
    double bottomTemperature = 0.0;
    double topCurrent = 0.0;
    double bottomCurrent = 0.0;
  }

  default void updateInputs(GripperIOInputs inputs) {}

  default void setVelocities(double topCommandVelocity, double bottomCommandVelocity) {}
}

package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    double position = 0.0;
    double velocity = 0.0;

    double appliedOutput = 0.0;
    double temperature = 0.0;
    double current = 0.0;

    boolean limitSwitch = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPosition(double commandPosition, double arbFF) {}
}

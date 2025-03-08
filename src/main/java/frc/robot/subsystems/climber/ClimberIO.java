package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog // needed for logging
  class ClimberIOInputs {
    double position = 0.0; // this is all the info needed for climber logs
    double velocity = 0.0;
    double temperature = 0.0;
    double current = 0.0;
    double appliedOutput = 0.0;

    double servoPosition = 0.0;

    // boolean limitSwitch = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setSpeed(double speed) {}

  default void setServo(double position) {}
}

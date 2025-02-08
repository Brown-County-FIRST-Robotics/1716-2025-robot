package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog // needed for logging
  class ClimberIOInputs {
    double positions[] = new double[] {0.0, 0.0}; // this is all the info needed for climber logs
    double velocities[] = new double[] {0.0, 0.0};
    double temperatures[] = new double[] {0.0, 0.0};
    double currents[] = new double[] {0.0, 0.0};
    double appliedOutputs[] = new double[] {0.0, 0.0};
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVelocities(double velocityLeft, double velocityRight) {}
}

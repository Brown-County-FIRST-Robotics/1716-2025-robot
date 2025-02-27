package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  // subsystem components such as motors:
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberIO io;

  private double positionOffset = 0.0;
  private boolean isDown = false;
  private boolean isZeroed = false;

  // Constructor
  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(
        () -> inputs.temperature, 60, 50, "climber motor"); // alerts if motor temps get too high
  }

  @Override
  public void periodic() { // runs every frame (useful for data logging)
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (inputs.limitSwitch) {
      isZeroed = true;
      positionOffset = inputs.position;
    }
    Logger.recordOutput("Climber/ActualPosition", inputs.position - positionOffset);

    if (isZeroed) {
      if (isDown) {
        io.setPosition(0.0); // TODO: set these to actual values
      } else {
        io.setPosition(1.0);
      }
    }
  }

  public void setPosition(boolean down) { // up or down
    isDown = down;
  }

  public void setVelocityFORZERO(
      double velocity) { // only to be used at robot initialization for zeroing
    io.setVelocity(velocity);
  }

  public boolean atLimit() {
    return inputs.limitSwitch;
  }
}

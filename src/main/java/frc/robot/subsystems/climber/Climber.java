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
  // private boolean isZeroed = false;

  // Constructor
  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(
        () -> inputs.temperature, 60, 50, "climber motor"); // alerts if motor temps get too high
    Logger.recordOutput("Climber/RequestedPosition", isDown);
  }

  @Override
  public void periodic() { // runs every frame (useful for data logging)
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    // if (inputs.limitSwitch) {
    //   isZeroed = true;
    //   positionOffset = inputs.position;
    // }
    Logger.recordOutput("Climber/ActualPosition", inputs.position - positionOffset);

    // if (isZeroed) {
    if (isDown) {
      io.setPosition(24 + positionOffset);
    } else {
      io.setPosition(0.5 + positionOffset);
    }
    // }
  }

  public void setPosition(boolean down) { // up or down
    isDown = down;
    Logger.recordOutput("Climber/RequestedPosition", isDown);
  }

  public void setVelocityFORZERO(
      double velocity) { // only to be used at robot initialization for zeroing
    io.setVelocity(velocity);
  }

  public void zero() { // Zero the position
    positionOffset = inputs.position;
  }

  public boolean atLimit() {
    return inputs.limitSwitch;
  }
}

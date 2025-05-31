package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  final ClimberIO io;

  private double positionOffset;
  private boolean isDown = false;

  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(() -> inputs.temperature, 60, 50, "climber motor");
    Logger.recordOutput("Climber/RequestedPosition", isDown);

    positionOffset = inputs.position;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/ActualPosition", inputs.position - positionOffset);
    if (isDown) {
      io.setPosition(92 + positionOffset);
    } else {
      io.setPosition(0.5 + positionOffset);
    }
  }

  /**
   * Commands a position
   *
   * @param isDown true if down, false if up
   */
  public void setPosition(boolean isDown) {
    this.isDown = isDown;
    Logger.recordOutput("Climber/RequestedPosition", this.isDown);
  }

  /**
   * Sets the commanded speed. Intended to be used during startup to find the limit switch.
   *
   * @param speed The commanded speed
   */
  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Sets the current position as the zero position */
  public void zero() {

    positionOffset = inputs.position;
  }

  public void setServo(boolean allowDown) {
    io.setServo(allowDown ? 180 : 0);
  }
}

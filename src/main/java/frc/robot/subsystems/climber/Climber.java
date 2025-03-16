package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  // subsystem components such as motors:
  final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  final ClimberIO io;

  private double positionOffset;
  private boolean isDown = false;

  // Constructor
  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(
        () -> inputs.temperature, 60, 50, "climber motor"); // alerts if motor temps get too high
    Logger.recordOutput("Climber/RequestedPosition", isDown);

    positionOffset = inputs.position;
  }

  @Override
  public void periodic() { // runs every frame (useful for data logging)
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/ActualPosition", inputs.position - positionOffset);

    if (isDown) {
      io.setPosition(
          80 + positionOffset); // TEMP: these need new values for the new gear ratio. They are
      // estimated to be 92 and 0.6
      // TESTME
    } else {
      io.setPosition(0.5 + positionOffset);
    }
  }

  public void setPosition(boolean down) { // up or down
    isDown = down;
    Logger.recordOutput("Climber/RequestedPosition", isDown);
  }

  public void setSpeedFORZERO(double speed) {
    io.setSpeed(speed);
  }

  public void zero() { // Zero the position

    positionOffset = inputs.position;
  }

  public void setServo(boolean allowDown) {
    io.setServo(allowDown ? 0 : 180); // NEEDS REAL VALUES
  }
}

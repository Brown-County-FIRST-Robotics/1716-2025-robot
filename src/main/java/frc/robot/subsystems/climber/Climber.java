package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  // subsystem components such as motors:
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberIO io;

  // Constructor
  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(
        () -> inputs.temperatures[0],
        60,
        50,
        "Left Climber motor"); // alerts if motor temps get too high
    CustomAlerts.makeOverTempAlert(() -> inputs.temperatures[1], 60, 50, "Right Climber motor");
  }

  @Override
  public void periodic() { // runs every frame (useful for data logging)
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}

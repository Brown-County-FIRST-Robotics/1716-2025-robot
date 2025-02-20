package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  // subsystem components such as motors:
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberIO io;

  private double leftPositionOffset = 0.0;
  private double rightPositionOffset = 0.0;

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

    if (inputs.limitSwitches[0]) {
      leftPositionOffset = inputs.positions[0];
    }
    if (inputs.limitSwitches[1]) {
      rightPositionOffset = inputs.positions[1];
    }
    Logger.recordOutput(
        "Climber/LeftActualPosition", inputs.positions[0] - leftPositionOffset);
    Logger.recordOutput(
        "Climber/RightActualPosition", inputs.positions[1] - rightPositionOffset);
  }

  public void setReference(double reference) {
    double convertedReference =
        Math.max(Math.min(reference, 0), 1.0); // prevent from going out of bounds
    // TODO: Update these values to the actual max and min based on the position of the limit switch

    double leftReference = convertedReference + leftPositionOffset;
    double rightReference = convertedReference + rightPositionOffset;

    Logger.recordOutput("Climber/CommandReference", reference);
    Logger.recordOutput("Climber/LeftActualReference", leftReference);
    Logger.recordOutput("Climber/RightActualReference", rightReference);

    io.setPositions(leftReference, rightReference);
  }
}

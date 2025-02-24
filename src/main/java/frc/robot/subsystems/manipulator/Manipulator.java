package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final WristIO wrist;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  private double elevatorPositionOffset = 0.0;

  public Manipulator(ElevatorIO elevator, WristIO wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public double getPos() {
    return elevatorInputs.position;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(elevatorInputs);
    wrist.updateInputs(wristInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.processInputs("Wrist", wristInputs);

    if (elevatorInputs.limitSwitch) {
      elevatorPositionOffset = elevatorInputs.position;
    }
    Logger.recordOutput(
        "Elevator/ActualPosition", elevatorInputs.position - elevatorPositionOffset);
  }

  public void setElevatorReference(double reference) {
    double convertedReference =
        Math.max(Math.min(reference, 0), 1.0); // prevent from going out of bounds
    // TODO: Update these values to the actual max and min based on the position of the limit switch
    convertedReference = convertedReference + elevatorPositionOffset;

    Logger.recordOutput("Elevator/CommandReference", reference);
    Logger.recordOutput("Elevator/ActualReference", convertedReference);
    elevator.setPosition(reference, 0);
  }

  public void setWristReference(double reference) {
    Logger.recordOutput("Wrist/Reference", reference);
    wrist.setPosition(Rotation2d.fromDegrees(reference), 0);
  }

  // Whether the arm is at the reference position (within some tolerance)
  public boolean isInPosition() {
    return Math.abs(elevatorInputs.height - elevatorCommandedPosition) < 0.02;
  }
}

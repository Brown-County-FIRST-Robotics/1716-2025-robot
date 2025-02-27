package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final WristIO wrist;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  private double elevatorCommandedPosition = 0.0;
  private double elevatorPositionOffset = 0.0;
  private double wristCommandedAngle = 0.0;

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
    elevatorCommandedPosition = reference;

    double convertedReference =
        Math.max(Math.min(reference, 0), 550.0); // prevent from going out of bounds
    // TODO: Update these values to the actual max and min based on the position of the limit switch
    convertedReference = convertedReference + elevatorPositionOffset;

    Logger.recordOutput("Elevator/CommandReference", reference);
    Logger.recordOutput("Elevator/ActualReference", convertedReference);
    elevator.setPosition(convertedReference, 0);
  }

  public double getW() {
    return wristInputs.angle;
  }

  public void setWristReference(double reference) {
    // double maxExtent =
    //     -180 * Math.pow(0.58 * elevatorInputs.position, 2)
    //         + 150; // https://www.desmos.com/calculator/qrehndsfwy
    // Max distance towards the electrical board the wrist can extend
    // maxExtent =
    //     Math.max(
    //         maxExtent * 1.0,
    //         10); // conversion from degrees to... and ensures above elevator hardware limit
    // double convertedReference =
    //     Math.max(Math.min(reference, 0), maxExtent); // prevent from going out of bounds

    double convertedReference =
        Math.max(Math.min(reference, 0), 1.0); // prevent from going out of bounds

    Logger.recordOutput("Wrist/CommandReference", reference);
    Logger.recordOutput("Wrist/ActualReference", convertedReference);
    wrist.setPosition(Rotation2d.fromDegrees(reference), 0);
  }

  // Whether the arm is at the reference position (within some tolerance)
  public boolean isInPosition() {
    return elevatorIsInPosition() && wristIsInPosiion();
  }

  public boolean elevatorIsInPosition() {
    return Math.abs(elevatorInputs.position - elevatorCommandedPosition) < 0.02;
  }

  public boolean wristIsInPosiion() {
    return Math.abs(wristInputs.angle - wristCommandedAngle) < 0.02;
  }
}

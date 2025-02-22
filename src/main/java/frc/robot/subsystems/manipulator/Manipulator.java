package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final WristIO wrist;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  private double elevatorCommandedPosition = 0;

  public Manipulator(ElevatorIO elevator, WristIO wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public double getPos() {
    return elevatorInputs.height;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(elevatorInputs);
    wrist.updateInputs(wristInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.processInputs("Wrist", wristInputs);
  }

  public void setElevatorReference(double reference) {
    Logger.recordOutput("Elevator/Reference", reference);
    elevatorCommandedPosition = reference;
    if (reference < 0) {
      reference = 0;
    }
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

package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final GripperIO gripper;
  private final GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();
  private final WristIO wrist;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  public Manipulator(ElevatorIO elevator, GripperIO gripper, WristIO wrist) {
    this.elevator = elevator;
    this.gripper = gripper;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(elevatorInputs);
    gripper.updateInputs(gripperInputs);
    wrist.updateInputs(wristInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.processInputs("Gripper", gripperInputs);
    Logger.processInputs("Wrist", wristInputs);
  }

  public void setElevatorReference(double reference) {
    Logger.recordOutput("Elevator/Reference", reference);
    elevator.setPosition(reference, 0);
  }

  public void setWristReference(double reference) {
    Logger.recordOutput("Wrist/Reference", reference);
    wrist.setPosition(Rotation2d.fromDegrees(reference), 0);
  }

  public void setGripper(double top, double bottom, double rear) {
    Logger.recordOutput("Gripper/TopReference", top);
    Logger.recordOutput("Gripper/BottomReference", bottom);
    Logger.recordOutput("Gripper/RearReference", rear);
    gripper.setVelocities(top, bottom, rear);
  }

  public Optional<Double> getDistanceReading() {
    return gripperInputs.hasLaserMeasurement
        ? Optional.of(gripperInputs.laserDistance)
        : Optional.empty();
  }

  //Whether the arm is at the reference position (within some tolerance)
  //TODO: implement
  public boolean isInPosition() {
    return true;
  }
}

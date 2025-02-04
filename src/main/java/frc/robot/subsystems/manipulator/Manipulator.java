package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void stopGripper() {
    Logger.recordOutput("Gripper/TopReference", 0.0);
    Logger.recordOutput("Gripper/BottomReference", 0.0);
    Logger.recordOutput("Gripper/RearReference", 0.0);
    gripper.setVelocities(0.0, 0.0, 0.0);
  }

  public void intake() {
    Logger.recordOutput("Gripper/TopReference", -0.5);
    Logger.recordOutput("Gripper/BottomReference", -0.5);
    Logger.recordOutput("Gripper/RearReference", -0.5);
    gripper.setVelocities(-0.5, -0.5, -0.5);
  }

  public void deposit() {
    Logger.recordOutput("Gripper/TopReference", 0.5);
    Logger.recordOutput("Gripper/BottomReference", 0.5);
    Logger.recordOutput("Gripper/RearReference", 0.5);
    gripper.setVelocities(0.5, 0.5, 0.5);
  }
}

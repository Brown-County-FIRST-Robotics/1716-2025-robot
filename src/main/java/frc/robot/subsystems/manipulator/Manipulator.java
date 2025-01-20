package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final GripperIO gripper;
  private final GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();
  private final WristIO wrist;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  public Manipulator() {
    elevator = new ElevatorIOSparkMax();
    gripper = new GripperIOSparkMax();
    wrist = new WristIOSparkFlex();
  }

  // TODO: Add logging
  public void setElevatorReference(double reference) {
    elevator.setPosition(reference, 0);
  }

  public void setWristReference(double reference) {
    wrist.setPosition(Rotation2d.fromDegrees(reference), 0);
  }

  public void stopGrabber() {
    gripper.setVelocities(0.0, 0.0, 0.0);
  }

  public void intake() {
    gripper.setVelocities(-0.5, -0.5, -0.5);
  }

  public void deposit() {
    gripper.setVelocities(0.5, 0.5, 0.5);
  }
}

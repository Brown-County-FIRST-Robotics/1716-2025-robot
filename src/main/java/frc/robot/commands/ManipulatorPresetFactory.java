package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorPresetFactory {
  Manipulator manipulator;

  public ManipulatorPresetFactory(Manipulator manipulator_) {
    manipulator = manipulator_;
  }

  // TODO: Set placeholder values
  // TODO: Add LoggedTunableNumbers
  public Command retracted() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(0.0);
          manipulator.setWristReference(0.0);
        },
        manipulator);
  }

  public Command trough() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command level2() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command level3() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command level4() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command algaeLow() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command algaeHigh() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command intake() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }

  public Command processor() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(1.0);
          manipulator.setWristReference(1.0);
        },
        manipulator);
  }
}

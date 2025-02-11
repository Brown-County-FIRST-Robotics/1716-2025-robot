package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.utils.LoggedTunableNumber;

public class ManipulatorPresetFactory {
  Manipulator manipulator;

  LoggedTunableNumber elevatorRetracted = new LoggedTunableNumber("Elevator Retracted", 0.0);
  LoggedTunableNumber wristRetracted = new LoggedTunableNumber("Wrist Retracted", 0.0);
  LoggedTunableNumber elevatorTrough = new LoggedTunableNumber("Elevator Trough", 1.0);
  LoggedTunableNumber wristTrough = new LoggedTunableNumber("Wrist Trough", 1.0);
  LoggedTunableNumber elevatorLevel2 = new LoggedTunableNumber("Elevator Level 2", 1.0);
  LoggedTunableNumber wristLevel2 = new LoggedTunableNumber("Wrist Level 2", 1.0);
  LoggedTunableNumber elevatorLevel3 = new LoggedTunableNumber("Elevator Level 3", 1.0);
  LoggedTunableNumber wristLevel3 = new LoggedTunableNumber("Wrist Level 3", 1.0);
  LoggedTunableNumber elevatorLevel4 = new LoggedTunableNumber("Elevator Level 4", 1.0);
  LoggedTunableNumber wristLevel4 = new LoggedTunableNumber("Wrist Level 4", 1.0);
  LoggedTunableNumber elevatorAlgaeLow = new LoggedTunableNumber("Elevator Algae Low", 1.0);
  LoggedTunableNumber wristAlgaeLow = new LoggedTunableNumber("Wrist Algae Low", 1.0);
  LoggedTunableNumber elevatorAlgaeHigh = new LoggedTunableNumber("Elevator Algae High", 1.0);
  LoggedTunableNumber wristAlgaeHigh = new LoggedTunableNumber("Wrist Algae High", 1.0);
  LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator Intake", 1.0);
  LoggedTunableNumber wristIntake = new LoggedTunableNumber("Wrist Intake", 1.0);
  LoggedTunableNumber elevatorProcessor = new LoggedTunableNumber("Elevator Processor", 1.0);
  LoggedTunableNumber wristProcessor = new LoggedTunableNumber("Wrist Processor", 1.0);

  public ManipulatorPresetFactory(Manipulator manipulator_) {
    manipulator = manipulator_;
  }

  public Command retracted() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorRetracted.get());
          manipulator.setWristReference(wristRetracted.get());
        },
        manipulator);
  }

  public Command trough() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorTrough.get());
          manipulator.setWristReference(wristTrough.get());
        },
        manipulator);
  }

  public Command level2() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorLevel2.get());
          manipulator.setWristReference(wristLevel2.get());
        },
        manipulator);
  }

  public Command level3() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorLevel3.get());
          manipulator.setWristReference(wristLevel3.get());
        },
        manipulator);
  }

  public Command level4() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorLevel4.get());
          manipulator.setWristReference(wristLevel4.get());
        },
        manipulator);
  }

  public Command algaeLow() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorAlgaeLow.get());
          manipulator.setWristReference(wristAlgaeLow.get());
        },
        manipulator);
  }

  public Command algaeHigh() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorAlgaeHigh.get());
          manipulator.setWristReference(wristAlgaeHigh.get());
        },
        manipulator);
  }

  // currently maintains control of position until the coral is properly positioned, slowing the
  // robot down
  public Command intake() {
    return Commands.runEnd(
            () -> {
              manipulator.setElevatorReference(elevatorIntake.get());
              manipulator.setWristReference(wristIntake.get());

              if (manipulator.isInPosition()) {
                manipulator.setGripper(-3500, -3500, -3500);
              } else {
                manipulator.setGripper(0, 0, 0);
              }
            },
            () -> manipulator.setGripper(0, 0, 0),
            manipulator)
        .until(
            () ->
                manipulator
                    .getDistanceReading()
                    .filter(
                        (Double d) -> {
                          return d < 0.1;
                        })
                    .isEmpty())
        .andThen(
            Commands.runEnd(
                    () -> manipulator.setGripper(1000, 1000, 1000),
                    () -> manipulator.setGripper(0, 0, 0),
                    manipulator)
                .until(
                    () ->
                        manipulator
                            .getDistanceReading()
                            .filter(
                                (Double d) -> {
                                  return d < 0.1;
                                })
                            .isPresent()));
  }

  public Command processor() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorProcessor.get());
          manipulator.setWristReference(wristProcessor.get());
        },
        manipulator);
  }
}

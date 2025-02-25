package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.buttonbox.ManipulatorPanel;
import java.util.Optional;

public class ManipulatorPresetFactory {
  Manipulator manipulator;
  Gripper gripper;
  TeleopDrive teleopDrive;
  Drivetrain driveTrain;
  ManipulatorPanel manipulatorPanel;

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
  LoggedTunableNumber elevatorProcessor = new LoggedTunableNumber("Elevator Processor", 1.0);
  LoggedTunableNumber wristProcessor = new LoggedTunableNumber("Wrist Processor", 1.0);

  // Intake
  LoggedTunableNumber elevatorBase = new LoggedTunableNumber("Elevator Intake", 1.0);
  LoggedTunableNumber wristOut = new LoggedTunableNumber("Wrist Intake", 1.0);
  LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator Intake", 1.0);
  LoggedTunableNumber wristIntake = new LoggedTunableNumber("Wrist Intake", 1.0);

  public ManipulatorPresetFactory(
      Manipulator manipulator_,
      Gripper gripper_,
      TeleopDrive teleopDrive_,
      Drivetrain driveTrain_,
      ManipulatorPanel manipulatorPanel_) {
    manipulator = manipulator_;
    gripper = gripper_;
    driveTrain = driveTrain_;
    teleopDrive = teleopDrive_;
    manipulatorPanel = manipulatorPanel_;
  }

  public Optional<Translation2d> whereShouldIBe() {
    var position = driveTrain.getPosition();
    for (int i = 0; i < 6; i++) {
      if (FieldConstants.getBox(i).intersects(position.getTranslation())) {
        return Optional.of(FieldConstants.getPole(i, manipulatorPanel.leftPole().getAsBoolean()));
      }
    }
    return Optional.empty();
  }

  public Command aim() {
    return Commands.runEnd(
        () ->
            teleopDrive.setCustomRotation(
                whereShouldIBe()
                    .map(
                        (Translation2d translate) ->
                            driveTrain.getPosition().getTranslation().minus(translate).getAngle())),
        () -> teleopDrive.setCustomRotation(Optional.empty()));
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

              if (manipulator.isInPosition()) {
                gripper.setGripper(-3500);
              } else {
                gripper.setGripper(0);
              }
            },
            gripper,
            manipulator)
        .until(
            () ->
                gripper
                    .getAlgaeDistanceReading()
                    .filter(
                        (Double d) -> {
                          return d < 0.1;
                        })
                    .isEmpty())
        .andThen(gripper.holdAlgae());
  }

  public Command algaeHigh() {
    return Commands.run(
            () -> {
              manipulator.setElevatorReference(elevatorAlgaeHigh.get());
              manipulator.setWristReference(wristAlgaeHigh.get());

              if (manipulator.isInPosition()) {
                gripper.setGripper(1000);
              } else {
                gripper.setGripper(0);
              }
            },
            gripper,
            manipulator)
        .until(
            () ->
                gripper
                    .getAlgaeDistanceReading()
                    .filter(
                        (Double d) -> {
                          return d < 0.1;
                        })
                    .isEmpty())
        .andThen(gripper.holdAlgae());
  }

  public Command intake() {
    return Commands.sequence(
        ////////////////
        // Lower elevator all the way
        ////////////////
        Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorBase.get());
                  manipulator.setWristReference(wristOut.get());
                },
                manipulator)
            .until(() -> manipulator.elevatorIsInPosition()),
        ////////////////////////////////
        // Flips wrist over
        //////////////////////////////
        Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorBase.get());
                  manipulator.setWristReference(wristIntake.get());
                },
                manipulator)
            .until(() -> manipulator.wristIsInPosiion()),
        //////////////////////////////
        // Raises elevator to ramp height
        //////////////////////////////
        Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorIntake.get());
                  manipulator.setWristReference(wristIntake.get());
                },
                manipulator)
            .until(() -> manipulator.elevatorIsInPosition()),
        //////////////////////////////
        // Holds the manipulator position and intakes
        //////////////////////////////
        Commands.runEnd(
                () -> {
                  manipulator.setElevatorReference(elevatorIntake.get());
                  manipulator.setWristReference(wristIntake.get());
                  gripper.setGripper(-3500);
                },
                () -> gripper.setGripper(0),
                manipulator,
                gripper)
            .until(
                () ->
                    gripper
                        .getCoralDistanceReading()
                        .filter(
                            (Double d) -> {
                              return d < 0.1;
                            })
                        .isPresent()),
        //////////////////////////////
        // No longer cares about button input
        //////////////////////////////
        new ScheduleCommand(
            ///////////////////////////////////////////////
            // Runs the gripper until it sees the coral
            ///////////////////////////////////////////////
            Commands.runEnd(
                    () -> {
                      gripper.setGripper(-3500);
                    },
                    () -> gripper.setGripper(0),
                    gripper)
                .until(
                    () ->
                        gripper
                            .getCoralDistanceReading()
                            .filter(
                                (Double d) -> {
                                  return d < 0.1;
                                })
                            .isPresent())
                .alongWith(
                    Commands.sequence(
                        //////////////////////////////
                        // Lower elevator
                        //////////////////////////////
                        Commands.run(
                                () -> {
                                  manipulator.setElevatorReference(elevatorBase.get());
                                  manipulator.setWristReference(wristIntake.get());
                                },
                                manipulator)
                            .until(() -> manipulator.elevatorIsInPosition())),
                    //////////////////////////////
                    // Extend wrist back out
                    //////////////////////////////
                    Commands.run(
                            () -> {
                              manipulator.setElevatorReference(elevatorBase.get());
                              manipulator.setWristReference(wristOut.get());
                            },
                            manipulator)
                        .until(() -> manipulator.wristIsInPosiion()))));
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

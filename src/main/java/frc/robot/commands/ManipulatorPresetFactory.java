package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.buttonbox.ManipulatorPanel;
import java.util.Optional;

public class ManipulatorPresetFactory {
  final Manipulator manipulator;
  final Gripper gripper;
  final TeleopDrive teleopDrive;
  final Drivetrain driveTrain;
  final ManipulatorPanel manipulatorPanel;
  final LEDs leds;

  final LoggedTunableNumber elevatorRetracted = new LoggedTunableNumber("Elevator Retracted", 0.0);
  final LoggedTunableNumber wristRetracted = new LoggedTunableNumber("Wrist Retracted", -.4);
  final LoggedTunableNumber elevatorTrough = new LoggedTunableNumber("Elevator Trough", 30);
  final LoggedTunableNumber wristTrough = new LoggedTunableNumber("Wrist Trough", -.34);
  final LoggedTunableNumber elevatorLevel2 = new LoggedTunableNumber("Elevator Level 2", 70);
  final LoggedTunableNumber wristLevel2 = new LoggedTunableNumber("Wrist Level 2", -.38);
  final LoggedTunableNumber elevatorLevel3 = new LoggedTunableNumber("Elevator Level 3", 120);
  final LoggedTunableNumber wristLevel3 = new LoggedTunableNumber("Wrist Level 3", -.4);
  final LoggedTunableNumber elevatorAlgaeLow = new LoggedTunableNumber("Elevator Algae Low", 85);
  final LoggedTunableNumber elevatorAlgaeHigh = new LoggedTunableNumber("Elevator Algae High", 127);
  final LoggedTunableNumber wristAlgae = new LoggedTunableNumber("Wrist Algae", -.41);
  final LoggedTunableNumber elevatorProcessor = new LoggedTunableNumber("Elevator Processor", 0);
  final LoggedTunableNumber wristProcessor = new LoggedTunableNumber("Wrist Processor", -.3);
  final LoggedTunableNumber elevatorIntake = new LoggedTunableNumber("Elevator Intake", 10.0);
  final LoggedTunableNumber wristIntake = new LoggedTunableNumber("Wrist Intake", -.0038);
  final LoggedTunableNumber safeWrist = new LoggedTunableNumber("Safe Wrist Place", -0.22);
  final LoggedTunableNumber wristIntakeDescending =
      new LoggedTunableNumber("Wrist Intake on Descent", -.4);

  public ManipulatorPresetFactory(
      Manipulator manipulator_,
      Gripper gripper_,
      TeleopDrive teleopDrive_,
      Drivetrain driveTrain_,
      ManipulatorPanel manipulatorPanel_,
      LEDs leds_) {
    manipulator = manipulator_;
    gripper = gripper_;
    driveTrain = driveTrain_;
    teleopDrive = teleopDrive_;
    manipulatorPanel = manipulatorPanel_;
    leds = leds_;
  }

  /**
   * Gets the pole the robot should drive to, based on location and L/R button
   *
   * @return The pose of the pole the robot should drive to
   */
  public Optional<Pose2d> targetPole() {
    var position = driveTrain.getPosition();
    for (int i = 0; i < 6; i++) {
      if (FieldConstants.getBox(i).contains(position.getTranslation())) {
        return Optional.of(FieldConstants.getPole(i, manipulatorPanel.leftPole().getAsBoolean()));
      }
    }
    return Optional.empty();
  }

  /**
   * Returns a command to retract the manipulator
   *
   * @return A command to retract the manipulator
   */
  public Command retracted() {
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorRetracted.get());
          manipulator.setWristReference(wristRetracted.get());
        },
        manipulator);
  }

  /**
   * Returns a command to go to a safe position, then the trough
   *
   * @return A command to go to a safe position, then the trough
   */
  public Command trough() {
    return Commands.run(() -> manipulator.setWristReference(safeWrist.get()), manipulator)
        .until(() -> manipulator.getWrist() - safeWrist.get() < 0.05)
        .andThen(
            Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorTrough.get());
                  manipulator.setWristReference(wristTrough.get());
                },
                manipulator));
  }

  /**
   * Returns a command to go to a safe position, then level 2
   *
   * @return A command to go to a safe position, then level 2
   */
  public Command level2() {
    return Commands.run(() -> manipulator.setWristReference(safeWrist.get()), manipulator)
        .until(() -> manipulator.getWrist() - safeWrist.get() < 0.05)
        .andThen(
            Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorLevel2.get());
                  manipulator.setWristReference(wristLevel2.get());
                },
                manipulator));
  }
  /**
   * Returns a command to a safe position, then level 3
   *
   * @return A command to a safe position, then level 3
   */
  public Command level3() {
    return Commands.run(() -> manipulator.setWristReference(safeWrist.get()), manipulator)
        .until(() -> manipulator.getWrist() - safeWrist.get() < 0.05)
        .andThen(
            Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorLevel3.get());
                  manipulator.setWristReference(wristLevel3.get());
                },
                manipulator));
  }

  public Command level(int level) {
    return switch (level) {
      case 1 -> trough();
      case 2 -> level2();
      case 3 -> level3();
      default -> Commands.none();
    };
  }
  /**
   * Returns a command to go to a safe position, then the lower algae position on the reef
   *
   * @return A command to go to a safe position, then the lower algae position on the reef
   */
  public Command algaeLow() {
    // return Commands.none();
    return Commands.run(() -> manipulator.setWristReference(safeWrist.get()), manipulator)
        .until(() -> manipulator.getWrist() - safeWrist.get() < 0.05)
        .andThen(
            Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorAlgaeLow.get());
                  manipulator.setWristReference(wristAlgae.get());
                },
                manipulator));
  }
  /**
   * Returns a command to go to a safe position, then the high algae position on the reef
   *
   * @return A command to go to a safe position, then the high algae position on the reef
   */
  public Command algaeHigh() {
    // return Commands.none();
    return Commands.run(() -> manipulator.setWristReference(safeWrist.get()), manipulator)
        .until(() -> manipulator.getWrist() - safeWrist.get() < 0.05)
        .andThen(
            Commands.run(
                () -> {
                  manipulator.setElevatorReference(elevatorAlgaeHigh.get());
                  manipulator.setWristReference(wristAlgae.get());
                },
                manipulator));
  }
  /**
   * Returns a modal command to go to intake position, spin up the gripper, and set the leds when in
   * position.
   *
   * @return A modal command to go to intake position, spin up the gripper, and set the leds when in
   *     position.
   */
  public Command intake() {
    return Commands.runEnd(
            () -> {
              manipulator.setElevatorReference(elevatorIntake.get());

              if (manipulator.getElevator() <= elevatorIntake.get() + 10) {
                manipulator.setWristReference(wristIntake.get());
                gripper.setGripper(3000);
              } else {
                manipulator.setWristReference(wristIntakeDescending.get());
                gripper.setGripper(0);
              }

              if (manipulator.isInPosition()) {
                leds.setColor(Color.kGreen);
              } else {
                leds.setColor(Color.kRed);
              }
            },
            () -> {
              gripper.setGripper(0);
              leds.setColor(Color.kRed);
            },
            manipulator,
            gripper)
        .until(() -> gripper.getCoralDistanceReading().filter((Double d) -> d < 0.04).isPresent());
  }
  /**
   * Returns a command to go to the processor
   *
   * @return A command to go to the processor
   */
  public Command processor() {
    // return Commands.none();
    return Commands.run(
        () -> {
          manipulator.setElevatorReference(elevatorProcessor.get());
          manipulator.setWristReference(wristProcessor.get());
        },
        manipulator);
  }
}

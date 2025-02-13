// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIONavx;
import frc.robot.subsystems.IMUIOPigeon;
import frc.robot.subsystems.IMUIOSim;
import frc.robot.subsystems.manipulator.*;
import frc.robot.subsystems.mecanum.*;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkFX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.buttonbox.ButtonBox;
import frc.robot.utils.buttonbox.ManipulatorPanel;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);
  private final ButtonBox buttonBox = new ButtonBox(2);
  private final OverridePanel overridePanel = new OverridePanel(buttonBox);
  private final ManipulatorPanel manipulatorPanel = new ManipulatorPanel(buttonBox);
  private final Drivetrain driveSys;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoFactory autoFactory;
  private final Manipulator manipulator;

  private final ManipulatorPresetFactory presetFactory;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    ElevatorIO elevatorIO = null;
    GripperIO gripperIO = null;
    WristIO wristIO = null;
    autoChooser = new LoggedDashboardChooser<Command>("Auto chooser");
    if (WhoAmI.mode != WhoAmI.Mode.REPLAY) {
      switch (WhoAmI.bot) {
        case MECHBASE:
          driveSys = new MecanumDrivetrain(new MecanumIOSpark(1, 2, 3, 4), new IMUIOPigeon(20));
          break;
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSim(0), 0),
                  new Module(new ModuleIOSim(1), 1),
                  new Module(new ModuleIOSim(2), 2),
                  new Module(new ModuleIOSim(3), 3),
                  new IMUIOSim());
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSparkFX(23, 31, "FL"), 0),
                  new Module(new ModuleIOSim(1), 1),
                  new Module(new ModuleIOSim(2), 2),
                  new Module(new ModuleIOSim(3), 3),
                  new IMUIOSim());
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(8 * 0.0254, 11 * 0.0254, 22 * 0.0254),
                        new Rotation3d(0, -8.0 * Math.PI / 180, 0))
                  },
                  new VisionIO[] {new VisionIOPhotonVision("SS_LAPTOP", "0")},
                  overridePanel);
          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIOSpark(1, 2, 3, 4), new IMUIONavx());
      }
      for (var appendage : WhoAmI.appendages) {
        if (appendage == WhoAmI.Appendages.GRIPPER) {
          gripperIO = new GripperIOSparkMax(31, 11, 4, 0);
        }
        System.out.println("No appendages yet");
      }
    } else {
      switch (WhoAmI.bot) {
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(0 * 0.0254, 0 * 0.0254, 22 * 0.0254),
                        new Rotation3d(0, -12 * Math.PI / 180, 0))
                  },
                  new VisionIO[] {new VisionIO() {}},
                  overridePanel);
          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIO() {}, new IMUIO() {});
      }
    }
    // TODO: Fix the followTrajectory code so it takes the correct argument (Collin)
    autoFactory = new AutoFactory(driveSys::getPosition, driveSys::setPosition, driveSys::followTrajectory, true, driveSys, new AutoBindings());
    autoChooser.addDefaultOption("Nothing", Commands.none());
    autoChooser.addOption(
        "Move backward 3s",
        Commands.runEnd(
                () -> driveSys.humanDrive(new ChassisSpeeds(-0.25, 0, 0)),
                () -> driveSys.humanDrive(new ChassisSpeeds()),
                driveSys)
            .raceWith(Commands.waitSeconds(3)));
    if (gripperIO == null) {
      gripperIO = new GripperIO() {};
    }
    if (wristIO == null) {
      wristIO = new WristIO() {};
    }
    if (elevatorIO == null) {
      elevatorIO = new ElevatorIO() {};
    }

    manipulator = new Manipulator(elevatorIO, gripperIO, wristIO);
    presetFactory = new ManipulatorPresetFactory(manipulator);

    // TODO: add appendage backups here
    TeleopDrive teleopDrive = configureSharedBindings();
    if (WhoAmI.isDemoMode) {
      configureDemoBindings(teleopDrive);
    } else {
      configureCompBindings();
    }
  }

  public void configureAutos() {}

  /** Updates the pose estimator to use the correct initial pose */
  public void setPose(Pose2d pose) {
    driveSys.setPosition(pose);
  }

  private void configureDemoBindings(TeleopDrive teleopDrive) {
    teleopDrive.isKidMode = false;
  }

  private void configureCompBindings() {
    // Manipulator Presets
    manipulator.setDefaultCommand(presetFactory.retracted());

    manipulatorPanel.trough().whileTrue(presetFactory.trough());
    manipulatorPanel.level2().whileTrue(presetFactory.level2());
    manipulatorPanel.level3().whileTrue(presetFactory.level3());
    manipulatorPanel.level4().whileTrue(presetFactory.level4());
    manipulatorPanel.algaeLow().whileTrue(presetFactory.algaeLow());
    manipulatorPanel.algaeHigh().whileTrue(presetFactory.algaeHigh());

    manipulatorPanel.intake().whileTrue(presetFactory.intake());
    manipulatorPanel.processor().whileTrue(presetFactory.processor());

    // Eject control on gripper, used for deposition, algae removal, and emergencies
    // Available to either driver
    driverController
        .rightTrigger(0.2)
        .or(manipulatorPanel.eject())
        .whileTrue(
            Commands.runEnd(
                () -> manipulator.setGripper(-2000, -2000, -2000),
                () -> manipulator.setGripper(0, 0, 0),
                manipulator));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private TeleopDrive configureSharedBindings() {
    var teleopDrive = new TeleopDrive(driveSys, driverController, secondController, overridePanel);
    driveSys.setDefaultCommand(teleopDrive);
    secondController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> teleopDrive.setKidModeSpeed(teleopDrive.getKidModeSpeed() + 0.5)));
    secondController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> teleopDrive.setKidModeSpeed(teleopDrive.getKidModeSpeed() - 0.5)));

    return teleopDrive;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

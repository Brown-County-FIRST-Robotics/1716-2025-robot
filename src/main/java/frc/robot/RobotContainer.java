// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManipulatorPresetFactory;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIONavx;
import frc.robot.subsystems.IMUIOPigeon;
import frc.robot.subsystems.IMUIOSim;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMaxes;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSparkMax;
import frc.robot.subsystems.manipulator.*;
import frc.robot.subsystems.manipulator.ElevatorIO;
import frc.robot.subsystems.manipulator.ElevatorIOSparkMax;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.WristIO;
import frc.robot.subsystems.mecanum.MecanumDrivetrain;
import frc.robot.subsystems.mecanum.MecanumIO;
import frc.robot.subsystems.mecanum.MecanumIOSpark;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkFX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.FusedVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionSLAMIOQuest;
import frc.robot.utils.buttonbox.ButtonBox;
import frc.robot.utils.buttonbox.ManipulatorPanel;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController secondController = new CommandXboxController(1);
  private final ButtonBox buttonBox = new ButtonBox(2);
  private final ManipulatorPanel manipulatorPanel = new ManipulatorPanel(buttonBox);
  private final OverridePanel overridePanel = new OverridePanel(buttonBox);
  private final Drivetrain driveSys;
  private final LEDs leds = new LEDs();
  private final LoggedDashboardChooser<Command> autoChooser;
  private AutoFactory autoFactory;
  private final Manipulator manipulator;
  private final Gripper gripper;
  public final Climber climber;

  Rotation3d rotation;
  Pose3d lpos = Pose3d.kZero;
  private final ManipulatorPresetFactory presetFactory;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    ElevatorIO elevatorIO = null;
    GripperIO gripperIO = null;
    WristIO wristIO = null;
    ClimberIO climberIO = null;
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
                  new Module(new ModuleIOSparkFX(24, 29, "FL"), 0),
                  new Module(new ModuleIOSparkFX(23, 19, "FR"), 1),
                  new Module(new ModuleIOSparkFX(20, 40, "BL"), 2),
                  new Module(new ModuleIOSparkFX(22, 9, "BR"), 3),
                  new IMUIONavx());
          var vision =
              new FusedVision(
                  driveSys,
                  new Transform3d(
                      new Translation3d(),
                      new Rotation3d(90.0 * Math.PI / 180.0, 0, 90.0 * Math.PI / 180.0)),
                  new VisionSLAMIOQuest(),
                  new VisionIOPhotonVision("TH_CAM0", new Transform3d()));

          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIOSpark(1, 2, 3, 4), new IMUIONavx());
      }
      for (var appendage : WhoAmI.appendages) {
        if (appendage == WhoAmI.Appendages.GRIPPER) {
          gripperIO = new GripperIOSparkMax(4, 1, 10, 0, 2);
        }
        if (appendage == WhoAmI.Appendages.ELEVATOR) {
          elevatorIO = new ElevatorIOSparkMax(53, 0);
        }
        if (appendage == WhoAmI.Appendages.CLIMBER) {
          climberIO = new ClimberIOSparkMaxes(36, 0); // TODO:Add real values
        }
        if (appendage == WhoAmI.Appendages.WRIST) {
          wristIO = new WristIOSparkFlex(55); // TODO:Add real values
        }
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
          // TEMP:ADD VISION BACK
          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIO() {}, new IMUIO() {});
      }
    }

    autoFactory =
        new AutoFactory(
            driveSys::getPosition,
            driveSys::setPosition, // TODO: don't do this (ie: give fake function)
            driveSys::followTrajectory,
            true,
            driveSys,
            new AutoFactory.AutoBindings());
    autoChooser.addDefaultOption("Nothing", Commands.none());
    autoChooser.addOption(
        "Move backward 3s",
        Commands.runEnd(
                () -> driveSys.humanDrive(new ChassisSpeeds(-1, 0, 0)),
                () -> driveSys.humanDrive(new ChassisSpeeds()),
                driveSys)
            .raceWith(Commands.waitSeconds(3)));

    // ************ SCORE 1 CORAL AND RETURN TO STATION ************
    // Routines for all 3 starting positions
    AutoRoutine lAuto = autoFactory.newRoutine("L-Auto");
    AutoRoutine mAuto = autoFactory.newRoutine("M-Auto");
    AutoRoutine rAuto = autoFactory.newRoutine("R-Auto");

    // Add all of the trajectories
    AutoTrajectory lAlign = lAuto.trajectory("L-Auto", 0);
    AutoTrajectory lPickup = lAuto.trajectory("L-Auto", 1);

    AutoTrajectory mAlign = mAuto.trajectory("M-Auto", 0);
    AutoTrajectory mPickup = mAuto.trajectory("M-Auto", 1);

    AutoTrajectory rAlign = rAuto.trajectory("R-Auto", 0);
    AutoTrajectory rPickup = rAuto.trajectory("R-Auto", 1);

    // Commands that start when auto routines are called
    lAuto.active().onTrue(lAlign.cmd().andThen(lPickup.cmd()));
    mAuto.active().onTrue(mAlign.cmd().andThen(mPickup.cmd()));
    rAuto.active().onTrue(rAlign.cmd().andThen(rPickup.cmd()));

    // Add paths to the auto chooser
    autoChooser.addOption("Left (Friendly) - Choreo", lAuto.cmd());
    autoChooser.addOption("Middle - Choreo", mAuto.cmd());
    autoChooser.addOption("Right (Opponent) - Choreo", rAuto.cmd());

    // ************ DRIVE TO CORAL STATION ************
    // Make the routines
    // They will drive to the nearest station, middle has an auto to go to either station
    AutoRoutine fToStation = autoFactory.newRoutine("L-ToStation");
    AutoRoutine mToStationL = autoFactory.newRoutine("M-ToStationL");
    AutoRoutine mToStationR = autoFactory.newRoutine("M-ToStationR");
    AutoRoutine rToStation = autoFactory.newRoutine("R-ToStation");

    // Get all of the trajectories from Choreo
    AutoTrajectory lToStationTraj = fToStation.trajectory("L-ToStation");
    AutoTrajectory mToStationTrajL = fToStation.trajectory("M-ToStationL");
    AutoTrajectory mToStationTrajR = fToStation.trajectory("M-ToStationR");
    AutoTrajectory rToStationTraj = fToStation.trajectory("R-ToStation");

    // Merge all the commands into the auto routines
    fToStation.active().onTrue(lToStationTraj.cmd());
    mToStationL.active().onTrue(mToStationTrajL.cmd());
    mToStationR.active().onTrue(mToStationTrajR.cmd());
    rToStation.active().onTrue(rToStationTraj.cmd());

    // Add the new paths to the auto chooser
    autoChooser.addOption("Left to station - Choreo", fToStation.cmd());
    autoChooser.addOption("Middle to left station - Choreo", mToStationL.cmd());
    autoChooser.addOption("Middle to right station - Choreo", mToStationR.cmd());
    autoChooser.addOption("Right to station - Choreo", rToStation.cmd());

    if (gripperIO == null) {
      gripperIO = new GripperIO() {};
    }
    if (wristIO == null) {
      wristIO = new WristIO() {};
    }
    if (elevatorIO == null) {
      elevatorIO = new ElevatorIO() {};
    }
    if (climberIO == null) {
      climberIO = new ClimberIO() {};
    }

    manipulator = new Manipulator(elevatorIO, wristIO);
    gripper = new Gripper(gripperIO);
    climber = new Climber(climberIO);

    // TODO: add appendage backups here
    TeleopDrive teleopDrive = configureSharedBindings();
    presetFactory =
        new ManipulatorPresetFactory(
            manipulator, gripper, teleopDrive, driveSys, manipulatorPanel, leds);
    autoChooser.addOption(
        "Crappy 1 coral",
        Commands.runEnd(
                () -> driveSys.humanDrive(new ChassisSpeeds(1, 0, 0)),
                () -> driveSys.humanDrive(new ChassisSpeeds()),
                driveSys)
            .alongWith(presetFactory.retracted())
            .raceWith(Commands.waitSeconds(2.0))
            .andThen(
                presetFactory
                    .level2()
                    .alongWith(
                        Commands.waitSeconds(3)
                            .andThen(
                                Commands.runEnd(
                                        () -> gripper.setGripper(-4000),
                                        () -> gripper.setGripper(0),
                                        gripper)
                                    .raceWith(Commands.waitSeconds(2))
                                    .andThen(
                                        Commands.runEnd(
                                                () ->
                                                    driveSys.humanDrive(
                                                        new ChassisSpeeds(-.5, 0, 0)),
                                                () -> driveSys.humanDrive(new ChassisSpeeds()),
                                                driveSys)
                                            .raceWith(Commands.waitSeconds(1.5)))))));

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

    // manipulator.setDefaultCommand(
    //     Commands.run(
    //         new Runnable() {

    //           public void run() {
    //             // manipulator.setWristReference(
    //             //     manipulator.getWrist() + driverController.getHID().getRightY());
    //             manipulator.setElevatorReference(
    //                 manipulator.getElevator()
    //                     + 50
    //                         * (-driverController.getLeftTriggerAxis()
    //                             + driverController.getRightTriggerAxis()));
    //           }
    //         },
    //         manipulator));

    manipulatorPanel.trough().whileTrue(presetFactory.trough());
    manipulatorPanel.level2().whileTrue(presetFactory.level2());
    manipulatorPanel.level3().whileTrue(presetFactory.level3());
    manipulatorPanel.level4().whileTrue(presetFactory.level4());
    manipulatorPanel
        .algaeLow()
        .whileTrue(presetFactory.algaeLow().alongWith(new ScheduleCommand(gripper.holdAlgae())));
    manipulatorPanel
        .algaeHigh()
        .whileTrue(presetFactory.algaeHigh().alongWith(new ScheduleCommand(gripper.holdAlgae())));

    manipulatorPanel.intake().and(() -> !gripper.hasAlgae()).onTrue(presetFactory.intake());
    manipulatorPanel.processor().and(gripper::hasAlgae).whileTrue(presetFactory.processor());

    manipulatorPanel.leftPole().or(manipulatorPanel.rightPole()).whileTrue(presetFactory.aim());

    manipulatorPanel
        .leftPole()
        .and(manipulatorPanel.rightPole())
        .onTrue(Commands.runOnce(() -> manipulator.resetElevator()));

    // Eject control on gripper, used for deposition, algae removal, and emergencies
    // Available to either driver
    driverController
        .rightTrigger(0.2)
        .or(driverController.leftTrigger(0.2))
        .or(manipulatorPanel.eject())
        .whileTrue(
            Commands.runEnd(() -> gripper.setGripper(-3000), () -> gripper.setGripper(0), gripper));

    driverController.back().onTrue(Commands.runOnce(() -> driveSys.setPosition(Pose2d.kZero)));

    // Climber
    driverController
        .a()
        .or(driverController.povDown())
        .onTrue(Commands.runOnce(() -> climber.setPosition(true), climber));
    driverController
        .y()
        .or(driverController.povUp())
        .onTrue(Commands.runOnce(() -> climber.setPosition(false), climber));
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
    // var teleopDrive = new TeleopDrive(driveSys, driverController, secondController,
    // overridePanel);
    var teleopDrive = new TeleopDrive(driveSys, driverController, overridePanel);

    driveSys.setDefaultCommand(teleopDrive);
    // secondController
    //     .povUp()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> teleopDrive.setKidModeSpeed(teleopDrive.getKidModeSpeed() + 0.5)));
    // secondController
    //     .povDown()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> teleopDrive.setKidModeSpeed(teleopDrive.getKidModeSpeed() - 0.5)));

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

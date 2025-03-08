package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Alert;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.PeriodicRunnable;
import java.io.File;
import java.io.FileNotFoundException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Scanner;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  boolean builtPoseSetter = false;
  private RobotContainer robotContainer;

  private final XboxController driverController = new XboxController(0);
  private boolean hasRumbledMatchTime = false; // hasStarted, hasEnded

  // TEMP CODE

  NetworkTable table = NetworkTableInstance.getDefault().getTable("questnav");
  DoubleArraySubscriber questRot =
      table.getDoubleArrayTopic("quaternion").subscribe(new double[] {});
  DoubleArraySubscriber questPosition =
      table.getDoubleArrayTopic("position").subscribe(new double[] {});
  Rotation3d rotation;

  // END TEMP CODE

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", "2025");
    File deployDir = Filesystem.getDeployDirectory();
    File tagFile = new File(deployDir, "git_tag.txt");
    File deployerFile = new File(deployDir, "deployer.txt");
    String tagName;
    String deployer;
    try {
      Scanner reader = new Scanner(tagFile);
      tagName = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      tagName = "Deploy did not send git data";
      new Alert(
              "Git data was not included in deploy. This will make it impossible to determine what code was run from the logfile. ",
              Alert.AlertType.WARNING)
          .set(true);
    }
    try {
      Scanner reader = new Scanner(deployerFile);
      deployer = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      new Alert("The identity of the deployer is unknown", Alert.AlertType.WARNING).set(true);
      deployer = "Unknown deployer";
    }
    Logger.recordMetadata("Tag Name", tagName);
    Logger.recordMetadata("Deployer", deployer);
    Logger.recordMetadata("Bot", String.valueOf(WhoAmI.bot));
    Logger.recordMetadata("Appendages", Arrays.toString(WhoAmI.appendages));
    Logger.recordMetadata("SN", HALUtil.getSerialNumber());

    switch (WhoAmI.mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U"));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("SimLogs/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(
                    logPath,
                    "_replay" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()))));
        break;
    }
    // Start AdvantageKit logger
    Logger.start();
    CustomAlerts.makeCANFailAlerts(0.9);
    robotContainer = new RobotContainer();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    PeriodicRunnable.runPeriodic();

    // TEMP CODE
    if (questRot.get().length > 0) {
      var q =
          new Quaternion(questRot.get()[0], questRot.get()[1], questRot.get()[2], questRot.get()[3])
              .toRotationVector();
      rotation = new Rotation3d(VecBuilder.fill(-q.get(0), -q.get(2), -q.get(1)));

      Logger.recordOutput(
          "headset/position",
          new Pose3d(
              questPosition.get()[0], questPosition.get()[1], questPosition.get()[2], rotation));
    }
    // END TEMP CODE

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SwerveSimManager.getInstance().propagate();
  }
}

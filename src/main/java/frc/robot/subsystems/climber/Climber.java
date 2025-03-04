package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  // subsystem components such as motors:
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberIO io;

  private double leftPositionOffset = 0.0;
  private double rightPositionOffset = 0.0;

  // Constructor
  public Climber(ClimberIO io) {
    this.io = io;
    CustomAlerts.makeOverTempAlert(
        () -> inputs.temperatures[0],
        60,
        50,
        "Left Climber motor"); // alerts if motor temps get too high
    CustomAlerts.makeOverTempAlert(() -> inputs.temperatures[1], 60, 50, "Right Climber motor");
  }

  @Override
  public void periodic() { // runs every frame (useful for data logging)
    var t0 = RobotController.getFPGATime();

    io.updateInputs(inputs);
    System.out.println("Climber Periodic:" + Long.toString(RobotController.getFPGATime() - t0));

    Logger.processInputs("Climber", inputs);

    if (inputs.limitSwitches[0]) {
      leftPositionOffset = inputs.positions[0];
    }
    if (inputs.limitSwitches[1]) {
      rightPositionOffset = inputs.positions[1];
    }
    Logger.recordOutput("Climber/LeftActualPosition", inputs.positions[0] - leftPositionOffset);
    Logger.recordOutput("Climber/RightActualPosition", inputs.positions[1] - rightPositionOffset);
  }

  public void setVelocity(double velocity) {
    double leftVelocity;
    double rightVelocity;

    // prevent it from overextending
    // TODO: set endpoints based on position of limit switch
    if ((inputs.positions[0] - leftPositionOffset > 0.0 || velocity > 0)
        && (inputs.positions[0] - leftPositionOffset < 1.0 || velocity < 0)) {
      leftVelocity = velocity;
    } else {
      leftVelocity = 0;
    }
    if ((inputs.positions[1] - rightPositionOffset > 0.0 || velocity > 0)
        && (inputs.positions[1] - rightPositionOffset < 1.0 || velocity < 0)) {
      rightVelocity = velocity;
    } else {
      rightVelocity = 0;
    }
    io.setVelocities(leftVelocity, rightVelocity);
  }

  public boolean atLimit() {
    return inputs.limitSwitches[0] && inputs.limitSwitches[1];
  }
}

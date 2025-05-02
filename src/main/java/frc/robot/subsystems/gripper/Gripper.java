package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO gripperIO;
  private final GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();
  private boolean hasAlgae = false;

  public Gripper(GripperIO gripper) {
    this.gripperIO = gripper;
  }

  @Override
  public void periodic() {
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Gripper", gripperInputs);
  }

  /**
   * Commands a speed to both motors
   *
   * @param speed The commanded speed in RPM
   */
  public void setGripper(double speed) {
    Logger.recordOutput("Gripper/TopReference", speed);
    Logger.recordOutput("Gripper/BottomReference", speed);
    gripperIO.setVelocities(speed, speed);
  }

  public Optional<Double> getCoralDistanceReading() {
    return gripperInputs.hasCoralLaserMeasurement
        ? Optional.of(gripperInputs.coralLaserDistance)
        : Optional.empty();
  }

  public Optional<Double> getAlgaeDistanceReading() {
    return gripperInputs.hasAlgaeLaserMeasurement
        ? Optional.of(gripperInputs.algaeLaserDistance)
        : Optional.empty();
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  // Whether the algae sensor can see anything close enough to be considered a gamepiece
  // NEEDS TESTING FOR CORAL PLACEMENT
  public boolean hasGamepiece() {
    return getAlgaeDistanceReading().filter((Double d) -> d < 0.15).isPresent();
  }

  /**
   * Returns a command to keep holding the algae
   *
   * @return Said command
   */
  public Command holdAlgae() {
    return Commands.runEnd(
            () -> {
              setGripper(4500);
              if (getAlgaeDistanceReading().filter((Double d) -> d < 0.15).isPresent()) {
                hasAlgae = true;
                Logger.recordOutput("Gripper/HasAlgae", true);
              }
            },
            () -> setGripper(0),
            this)
        .finallyDo(
            () -> {
              hasAlgae = false;
              Logger.recordOutput("Gripper/HasAlgae", false);
            })
        .until(
            () -> hasAlgae && getAlgaeDistanceReading().filter((Double d) -> d < 0.15).isEmpty());
  }
}

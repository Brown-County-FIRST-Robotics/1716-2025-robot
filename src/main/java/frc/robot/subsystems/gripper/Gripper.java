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
    Logger.recordOutput("Gripper/HasAlgae", hasAlgae);
  }

  public void setGripper(double top, double bottom, double rear) {
    Logger.recordOutput("Gripper/TopReference", top);
    Logger.recordOutput("Gripper/BottomReference", bottom);
    Logger.recordOutput("Gripper/RearReference", rear);
    gripperIO.setVelocities(top, bottom, rear);
  }

  public void setGripper(double speed) {
    Logger.recordOutput("Gripper/TopReference", speed);
    Logger.recordOutput("Gripper/BottomReference", speed);
    Logger.recordOutput("Gripper/RearReference", speed);
    gripperIO.setVelocities(speed, speed, speed);
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

  public Command holdAlgae() {
    return Commands.run(
            () -> {
              setGripper(getAlgaeDistanceReading().orElse(0.0) * 1000);
            },
            this)
        .until(() -> getAlgaeDistanceReading().isEmpty());
  }
}

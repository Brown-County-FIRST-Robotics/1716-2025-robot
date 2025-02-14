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

  public void setGripper(double speed) {
    Logger.recordOutput("Gripper/TopReference", speed);
    Logger.recordOutput("Gripper/BottomReference", speed);
    Logger.recordOutput("Gripper/RearReference", speed);
    gripperIO.setVelocities(speed, speed, speed);
  }

  public void setFront(double speed) {
    Logger.recordOutput("Gripper/TopReference", speed);
    Logger.recordOutput("Gripper/BottomReference", speed);
    Logger.recordOutput("Gripper/RearReference", speed);
    gripperIO.setVelocities(speed, speed, 0);
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

  public Command holdAlgae() {
    return Commands.run(
            () -> {
              // if (getAlgaeDistanceReading().orElse(0.15) > 0.11) {
              //   setFront(1500);
              // }
              // else {
              //   setFront(0);
              // }
              setFront((getAlgaeDistanceReading().orElse(0.15) - 0.11) * 50000);
              // setFront(2000);
              //System.out.println(getAlgaeDistanceReading().orElse(0.15));
            },
            this)
        .beforeStarting(
            () -> {
              hasAlgae = true;
              Logger.recordOutput("Gripper/HasAlgae", true);
            })
        .finallyDo(
            () -> {
              setFront(0);
              hasAlgae = false;
              Logger.recordOutput("Gripper/HasAlgae", false);
            })
        .until(
            () ->
                getAlgaeDistanceReading()
                    .filter(
                        (Double d) -> {
                          return d < 0.25;
                        })
                    .isEmpty());
  }
}

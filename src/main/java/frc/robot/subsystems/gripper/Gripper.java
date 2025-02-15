package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO gripper;
  private final GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();

  public Gripper(GripperIO gripper) {
    this.gripper = gripper;
  }

  @Override
  public void periodic() {
    gripper.updateInputs(gripperInputs);
    Logger.processInputs("Gripper", gripperInputs);
  }

  public void setGripper(double top, double bottom, double rear) {
    Logger.recordOutput("Gripper/TopReference", top);
    Logger.recordOutput("Gripper/BottomReference", bottom);
    Logger.recordOutput("Gripper/RearReference", rear);
    gripper.setVelocities(top, bottom, rear);
  }

  public Optional<Double> getDistanceReading() {
    return gripperInputs.hasLaserMeasurement
        ? Optional.of(gripperInputs.laserDistance)
        : Optional.empty();
  }
}

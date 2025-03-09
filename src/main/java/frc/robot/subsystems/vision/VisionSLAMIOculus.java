package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSLAMIOculus implements VisionSLAMIO {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("questnav");
  DoubleArraySubscriber questRot =
      table.getDoubleArrayTopic("quaternion").subscribe(new double[] {});
  DoubleArraySubscriber questPosition =
      table.getDoubleArrayTopic("position").subscribe(new double[] {});

  @Override
  public void updateInputs(VisionSLAMIOInputs inputs) {
    if (questRot.get().length > 0 && questPosition.get().length > 0) {
      inputs.present = true;
      inputs.questTranslation =
          new Translation3d(questPosition.get()[0], questPosition.get()[1], questPosition.get()[2]);
      inputs.questQuat =
          new Quaternion(
              questRot.get()[0], questRot.get()[1], questRot.get()[2], questRot.get()[3]);
    } else {
      inputs.present = false;
    }
  }

  @Override
  public void ADBKill() {
    VisionSLAMIO.super.ADBKill();
  }

  @Override
  public void ADBStart() {
    try {
      Runtime.getRuntime().exec(new String[] {"asd", "asdsa"});
    } catch (Exception e) {

    }
  }
}

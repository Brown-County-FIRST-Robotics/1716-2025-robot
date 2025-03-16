package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;

public class VisionSLAMIOQuest implements VisionSLAMIO {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("questnav");
  DoubleArraySubscriber questRot =
      table.getDoubleArrayTopic("quaternion").subscribe(new double[] {});
  DoubleArraySubscriber questPosition =
      table.getDoubleArrayTopic("position").subscribe(new double[] {});
  IntegerSubscriber questFrames = table.getIntegerTopic("frameCount").subscribe(0);
  DoubleSubscriber battery = table.getDoubleTopic("batteryPercent").subscribe(0);

  @Override
  public void updateInputs(VisionSLAMIOInputs inputs) {
    if (questRot.get().length > 0 && questPosition.get().length > 0) {
      inputs.present = true;
      inputs.questTranslation =
          new Translation3d(questPosition.get()[0], questPosition.get()[1], questPosition.get()[2]);
      inputs.questQuat =
          new Quaternion(
              questRot.get()[0], questRot.get()[1], questRot.get()[2], questRot.get()[3]);
      inputs.frames = questFrames.get();
    } else {
      inputs.present = false;
    }
    inputs.battPercent = battery.get();
  }
}

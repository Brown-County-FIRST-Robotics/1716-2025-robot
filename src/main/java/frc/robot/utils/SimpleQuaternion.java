package frc.robot.utils;

import edu.wpi.first.math.geometry.Quaternion;

public class SimpleQuaternion extends Quaternion {
  public SimpleQuaternion(double x, double y, double z, double w) {
    super(x, y, z, w);
  }

  public double getRoll() {
    return toRotationVector().get(1) * (180 / Math.PI);
  }

  public double getPitch() {
    return toRotationVector().get(2) * (180 / Math.PI);
  }

  public double getYaw() {
    return toRotationVector().get(0) * (180 / Math.PI);
  }

  public double getRollRadians() {
    return toRotationVector().get(1);
  }

  public double getPitchRadians() {
    return toRotationVector().get(2);
  }

  public double getYawRadians() {
    return toRotationVector().get(0);
  }
}

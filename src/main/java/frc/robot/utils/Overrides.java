package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Overrides {
  public static final LoggedNetworkBoolean useFieldOriented =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Use Field Oriented", true);
  public static final LoggedNetworkBoolean resetYaw =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Reset Yaw", false);
  public static final LoggedNetworkBoolean disableIMU =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable IMU", false);
  public static final LoggedNetworkBoolean disableVision =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Vision", false);
  public static final LoggedNetworkBoolean disableAutoAiming =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Auto Aiming", false);
  public static final LoggedNetworkBoolean disableAutoAlign =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Auto Align", false);

  // intake from floor, intake from source, aim for amp, aim for speaker:
  public static final LoggedNetworkBoolean disableArmAnglePresets =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Arm Angle Presets", false);
  public static final LoggedTunableNumber armAngleOverrideIncrementScale =
      new LoggedTunableNumber("Shuffleboard/Overrides/arm angle override increment scale", 0.1);
  public static final LoggedTunableNumber kitbot = new LoggedTunableNumber("kitbot", 60);
}

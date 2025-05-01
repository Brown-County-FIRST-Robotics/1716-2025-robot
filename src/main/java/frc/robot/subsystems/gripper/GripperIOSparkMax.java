package frc.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class GripperIOSparkMax implements GripperIO {
  private final SparkMax top;
  private final RelativeEncoder topEncoder;
  private final SparkMax bottom;
  private final RelativeEncoder bottomEncoder;
  private final LaserCan coralLaserCan;
  private LaserCan.Measurement coralMeasurement;

  public GripperIOSparkMax(int topID, int bottomID, int coralLaserID) {
    top = new SparkMax(topID, MotorType.kBrushless);
    topEncoder = top.getEncoder();
    bottom = new SparkMax(bottomID, MotorType.kBrushless);
    bottomEncoder = bottom.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    coralLaserCan = new LaserCan(coralLaserID);
    coralMeasurement = null; // set in UpdateInputs

    config.closedLoop.smartMotion.maxAcceleration(12000);
    config.smartCurrentLimit(Constants.CurrentLimits.NEO550).idleMode(IdleMode.kBrake);
    config.closedLoop.p(0.0001).i(0).d(0).maxOutput(1).minOutput(-1);
    config.closedLoop.velocityFF(1.0 / 12000);
    config.inverted(false);

    top.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    bottom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // LaserCan Configuration
    try {
      coralLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      // Configures which of the sensor diodes in the 16x16 sensor array are enabled
      coralLaserCan.setRegionOfInterest(
          new LaserCan.RegionOfInterest(
              8, 8, 16, 16)); // Defines a 16x16 rectangle at (8, 8), the center
      coralLaserCan.setTimingBudget(
          LaserCan.TimingBudget.TIMING_BUDGET_33MS); // Higher is more accurate but updates slower
    } catch (ConfigurationFailedException e) {
      new Alert("Coral LaserCan failed to start", frc.robot.utils.Alert.AlertType.ERROR).set(true);
    }
  }

  public void updateInputs(GripperIOInputs inputs) {
    inputs.topPosition = topEncoder.getPosition();
    inputs.topVelocity = topEncoder.getVelocity();
    inputs.bottomPosition = bottomEncoder.getPosition();
    inputs.bottomVelocity = bottomEncoder.getVelocity();

    inputs.topAppliedOutput = top.getAppliedOutput();
    inputs.topTemperature = top.getMotorTemperature();
    inputs.topCurrent = top.getOutputCurrent();
    inputs.bottomAppliedOutput = bottom.getAppliedOutput();
    inputs.bottomTemperature = bottom.getMotorTemperature();
    inputs.bottomCurrent = bottom.getOutputCurrent();

    coralMeasurement = coralLaserCan.getMeasurement();
    // check if lasercan currently has a valid measurement
    if (coralMeasurement != null
        && coralMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.hasCoralLaserMeasurement = true;
      inputs.coralLaserDistance = coralMeasurement.distance_mm / 1000.0;
    } else {
      inputs.hasCoralLaserMeasurement = false;
    }
  }

  public void setVelocities(double topCommandVelocity, double bottomCommandVelocity) {
    top.getClosedLoopController().setReference(topCommandVelocity, ControlType.kVelocity);
    bottom.getClosedLoopController().setReference(bottomCommandVelocity, ControlType.kVelocity);
  }
}

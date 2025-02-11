package frc.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class GripperIOSparkMax implements GripperIO {
  private final SparkMax top;
  private final SparkMaxConfig topConfig;
  private final RelativeEncoder topEncoder;
  private final SparkMax bottom;
  private final RelativeEncoder bottomEncoder;
  private final SparkMax rear;
  private final RelativeEncoder rearEncoder;
  private final LaserCan coralLaserCan;
  private final LaserCan algaeLaserCan;
  private LaserCan.Measurement coralMeasurement;
  private LaserCan.Measurement algaeMeasurement;

  public GripperIOSparkMax(int topID, int bottomID, int rearID, int coralLaserID, int algaeLaserID) {
    top = new SparkMax(topID, MotorType.kBrushless);
    topConfig = new SparkMaxConfig();
    topEncoder = top.getEncoder();
    bottom = new SparkMax(bottomID, MotorType.kBrushless);
    bottomEncoder = bottom.getEncoder();
    rear = new SparkMax(rearID, MotorType.kBrushless);
    rearEncoder = rear.getEncoder();
    coralLaserCan = new LaserCan(coralLaserID);
    algaeLaserCan = new LaserCan(algaeLaserID);
    coralMeasurement = null; // set in UpdateInputs
    algaeMeasurement = null;

    topConfig.closedLoop.maxMotion.maxAcceleration(12000); // placeholder
    topConfig.smartCurrentLimit(Constants.CurrentLimits.NEO550);
    topConfig.closedLoop.velocityFF(1.0 / 11000.0);

    top.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottom.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rear.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    top.setInverted(true);

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
      new Alert("LaserCan failed to start", frc.robot.utils.Alert.AlertType.ERROR).set(true);
    }
    
    try {
      algaeLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      // Configures which of the sensor diodes in the 16x16 sensor array are enabled
      algaeLaserCan.setRegionOfInterest(
          new LaserCan.RegionOfInterest(
              8, 8, 16, 16)); // Defines a 16x16 rectangle at (8, 8), the center
      algaeLaserCan.setTimingBudget(
          LaserCan.TimingBudget.TIMING_BUDGET_33MS); // Higher is more accurate but updates slower
    } catch (ConfigurationFailedException e) {
      new Alert("LaserCan failed to start", frc.robot.utils.Alert.AlertType.ERROR).set(true);
    }
  }

  public void updateInputs(GripperIOInputs inputs) {
    inputs.topPosition = topEncoder.getPosition();
    inputs.topVelocity = topEncoder.getVelocity();
    inputs.bottomPosition = bottomEncoder.getPosition();
    inputs.bottomVelocity = bottomEncoder.getVelocity();
    inputs.rearPosition = rearEncoder.getPosition();
    inputs.rearVelocity = rearEncoder.getVelocity();

    inputs.topAppliedOutput = top.getAppliedOutput();
    inputs.topTemperature = top.getMotorTemperature();
    inputs.topCurrent = top.getOutputCurrent();
    inputs.bottomAppliedOutput = bottom.getAppliedOutput();
    inputs.bottomTemperature = bottom.getMotorTemperature();
    inputs.bottomCurrent = bottom.getOutputCurrent();
    inputs.rearAppliedOutput = rear.getAppliedOutput();
    inputs.rearTemperature = rear.getMotorTemperature();
    inputs.rearCurrent = rear.getOutputCurrent();

    coralMeasurement = coralLaserCan.getMeasurement();
    algaeMeasurement = algaeLaserCan.getMeasurement();
    // check if lasercan currently has a valid measurment
    if (coralMeasurement != null && coralMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.hasCoralLaserMeasurement = true;
      inputs.coralLaserDistance = coralMeasurement.distance_mm / 1000.0;
    } else {
      inputs.hasCoralLaserMeasurement = false;
    }
    
    if (algaeLaserCan != null && algaeMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.hasAlgaeLaserMeasurement = true;
      inputs.algaeLaserDistance = algaeMeasurement.distance_mm / 1000.0;
    } else {
      inputs.hasAlgaeLaserMeasurement = false;
    }
  }

  public void setVelocities(
      double topCommandVelocity, double bottomCommandVelocity, double rearCommandVelocity) {
    top.getClosedLoopController()
        .setReference(topCommandVelocity, ControlType.kMAXMotionVelocityControl);
    bottom
        .getClosedLoopController()
        .setReference(bottomCommandVelocity, ControlType.kMAXMotionVelocityControl);
    rear.getClosedLoopController()
        .setReference(rearCommandVelocity, ControlType.kMAXMotionVelocityControl);
  }
}

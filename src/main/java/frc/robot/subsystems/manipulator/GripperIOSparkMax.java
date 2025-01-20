package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class GripperIOSparkMax implements GripperIO {
  private final SparkMax top = new SparkMax(0, MotorType.kBrushless);
  private final SparkMaxConfig topConfig = new SparkMaxConfig();
  private final RelativeEncoder topEncoder = top.getEncoder();
  private final SparkMax bottom = new SparkMax(1, MotorType.kBrushless);
  private final SparkMaxConfig bottomConfig = new SparkMaxConfig();
  private final RelativeEncoder bottomEncoder = bottom.getEncoder();
  private final SparkMax rear = new SparkMax(2, MotorType.kBrushless);
  private final SparkMaxConfig rearConfig = new SparkMaxConfig();
  private final RelativeEncoder rearEncoder = rear.getEncoder();

  public GripperIOSparkMax() {
    topConfig.closedLoop.maxMotion.maxAcceleration(12000); // placeholder
    bottomConfig.closedLoop.maxMotion.maxAcceleration(12000);
    rearConfig.closedLoop.maxMotion.maxAcceleration(12000);
    topConfig.smartCurrentLimit(Constants.CurrentLimits.NEO550);
    bottomConfig.smartCurrentLimit(Constants.CurrentLimits.NEO550);
    rearConfig.smartCurrentLimit(Constants.CurrentLimits.NEO550);

    top.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottom.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rear.configure(rearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

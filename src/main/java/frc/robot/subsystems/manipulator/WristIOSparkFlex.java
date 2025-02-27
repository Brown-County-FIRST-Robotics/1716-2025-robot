package frc.robot.subsystems.manipulator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class WristIOSparkFlex implements WristIO {
  private final SparkFlex wrist;
  private final SparkFlexConfig wristConfig;
  private final AbsoluteEncoder encoder;
  private final double offset = 0.0;

  public WristIOSparkFlex(int id) {
    wrist = new SparkFlex(id, MotorType.kBrushless);
    wristConfig = new SparkFlexConfig();
    encoder = wrist.getAbsoluteEncoder();
    wristConfig.closedLoop.velocityFF(1.0 / 6700.0).maxOutput(1).minOutput(-1);
    wristConfig.closedLoop.smartMotion.maxAcceleration(1200); // placeholder
    wristConfig.smartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.angle = encoder.getPosition() - offset;
    inputs.omega = encoder.getVelocity();

    inputs.appliedOutput = wrist.getAppliedOutput();
    inputs.temperature = wrist.getMotorTemperature();
    inputs.current = wrist.getOutputCurrent();
  }

  public void setPosition(double commandPosition, double arbFF) {
    wrist
        .getClosedLoopController()
        .setReference(commandPosition + offset, ControlType.kSmartMotion);
  }
}

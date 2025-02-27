package frc.robot.subsystems.manipulator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class WristIOSparkFlex implements WristIO {
  private final SparkFlex wrist;
  private final SparkFlexConfig wristConfig;
  private final AbsoluteEncoder encoder;

  public WristIOSparkFlex(int id, int absEncoderID) {
    wrist = new SparkFlex(id, MotorType.kBrushless);
    wristConfig = new SparkFlexConfig();
    encoder = wrist.getAbsoluteEncoder();

    wristConfig.closedLoop.maxMotion.maxAcceleration(1200); // placeholder
    wristConfig.smartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(encoder.getPosition());
    inputs.omega = encoder.getVelocity();

    inputs.appliedOutput = wrist.getAppliedOutput();
    inputs.temperature = wrist.getMotorTemperature();
    inputs.current = wrist.getOutputCurrent();
  }

  public void setPosition(double commandPosition, double arbFF) {
    wrist
        .getClosedLoopController()
        .setReference(
            commandPosition, ControlType.kPosition);
  }
}

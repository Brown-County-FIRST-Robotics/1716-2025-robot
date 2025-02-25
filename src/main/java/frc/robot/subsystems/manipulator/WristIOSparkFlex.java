package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class WristIOSparkFlex implements WristIO {
  private final SparkFlex wrist;
  private final SparkFlexConfig wristConfig;
  private final RelativeEncoder relativeEncoder;
  private final DutyCycleEncoder absoluteEncoder;

  public WristIOSparkFlex(int id, int absEncoderID) {
    wrist = new SparkFlex(id, MotorType.kBrushless);
    wristConfig = new SparkFlexConfig();
    relativeEncoder = wrist.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(absEncoderID);

    wristConfig.closedLoop.maxMotion.maxAcceleration(1200); // placeholder
    wristConfig.smartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(absoluteEncoder.get());
    inputs.omega = relativeEncoder.getVelocity();

    inputs.appliedOutput = wrist.getAppliedOutput();
    inputs.temperature = wrist.getMotorTemperature();
    inputs.current = wrist.getOutputCurrent();
  }

  public void setPosition(double commandPosition, double arbFF) {
    // TODO: use absolute encoder
    wrist
        .getClosedLoopController()
        .setReference(
            commandPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFF);
  }
}

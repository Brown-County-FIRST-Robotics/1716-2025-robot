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
import frc.robot.Constants;

public class WristIOSparkFlex implements WristIO {
  private final SparkFlex wrist = new SparkFlex(0, MotorType.kBrushless);
  private final SparkFlexConfig wristConfig = new SparkFlexConfig();
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  public WristIOSparkFlex() {
    wristConfig.closedLoop.maxMotion.maxAcceleration(1200); // placeholder
    wristConfig.smartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(wristEncoder.getPosition());
    inputs.omega = wristEncoder.getVelocity();

    inputs.appliedOutput = wrist.getAppliedOutput();
    inputs.temperature = wrist.getMotorTemperature();
    inputs.current = wrist.getOutputCurrent();
  }

  public void setPosition(double commandPosition, double arbFF) {
    wrist
        .getClosedLoopController()
        .setReference(
            commandPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFF);
  }
}

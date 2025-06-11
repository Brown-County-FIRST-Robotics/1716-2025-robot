package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkFlex elevator_primary;
  private final SparkFlex elevator_follower;
  private final RelativeEncoder elevatorEncoder;

  public ElevatorIOSparkMax(int id, int follower_id) {
    elevator_primary = new SparkFlex(id, MotorType.kBrushless);
    elevator_follower = new SparkFlex(follower_id, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorEncoder = elevator_primary.getEncoder();

    elevatorConfig
        .closedLoop
        .smartMotion
        .maxAcceleration(6000)
        .maxVelocity(6000)
        .minOutputVelocity(0);
    elevatorConfig
        .closedLoop
        .velocityFF(1.0 / 6700.0)
        .p(1.0 / 6700.0)
        .maxOutput(1)
        .minOutput(-.5)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig
        .smartCurrentLimit(
            60) // Lesser current limit to prevent elevator mechanically falling apart
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    elevator_primary.configure(
        elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig follower = new SparkMaxConfig();
    follower.follow(elevator_primary, true);
    follower.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    elevator_follower.configure(
        follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = elevatorEncoder.getPosition();
    inputs.velocity = elevatorEncoder.getVelocity();

    inputs.appliedOutput = elevator_primary.getAppliedOutput();
    inputs.temperature_1 = elevator_primary.getMotorTemperature();
    inputs.temperature_2 = elevator_follower.getMotorTemperature();
    inputs.current = elevator_primary.getOutputCurrent();
  }

  public void setPosition(double commandPosition, double arbFF) {
    elevator_primary
        .getClosedLoopController()
        .setReference(commandPosition, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0, 0.2);
  }
}

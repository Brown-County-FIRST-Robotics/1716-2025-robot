package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkFlex elevator;
  private final RelativeEncoder elevatorEncoder;
  private final SparkLimitSwitch limitSwitch;

  public ElevatorIOSparkMax(int id, int limitSwitchID) {
    elevator = new SparkFlex(id, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorEncoder = elevator.getEncoder();
    limitSwitch = elevator.getReverseLimitSwitch();

    elevatorConfig
        .closedLoop
        .smartMotion
        .maxAcceleration(6000)
        .maxVelocity(6000)
        .minOutputVelocity(0); // placeholder
    elevatorConfig
        .closedLoop
        .velocityFF(1.0 / 6700.0)
        .p(1.0 / 6700.0)
        .maxOutput(1)
        .minOutput(-.5)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig
        .smartCurrentLimit(60) // Constants.CurrentLimits.NEO_VORTEX)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    // elevatorConfig.softLimit.forwardSoftLimitEnabled(false).forwardSoftLimit(182.0);
    elevator.configure(
        elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = elevatorEncoder.getPosition();
    inputs.velocity = elevatorEncoder.getVelocity();

    inputs.appliedOutput = elevator.getAppliedOutput();
    inputs.temperature = elevator.getMotorTemperature();
    inputs.current = elevator.getOutputCurrent();
    inputs.limitSwitch = limitSwitch.isPressed();
  }

  public void setPosition(double commandPosition, double arbFF) {
    elevator
        .getClosedLoopController()
        .setReference(commandPosition, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0, 0.2);
  }
}

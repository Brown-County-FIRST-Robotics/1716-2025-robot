package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkFlex elevator;
  private final SparkMaxConfig elevatorConfig;
  private final RelativeEncoder elevatorEncoder;
  private final SparkLimitSwitch limitSwitch;

  public ElevatorIOSparkMax(int id, int limitSwitchID) {
    elevator = new SparkFlex(id, MotorType.kBrushless);
    elevatorConfig = new SparkMaxConfig();
    elevatorEncoder = elevator.getEncoder();
    limitSwitch = elevator.getForwardLimitSwitch();

    elevatorConfig
        .closedLoop
        .smartMotion
        .maxAcceleration(1200)
        .maxVelocity(6700)
        .minOutputVelocity(0); // placeholder
    elevatorConfig
        .closedLoop
        .velocityFF(1.0 / 6700.0)
        .p(1.0 / 6700.0)
        .maxOutput(1)
        .minOutput(-1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig.smartCurrentLimit(Constants.CurrentLimits.NEO);
    // top447   -97
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
    elevator.getClosedLoopController().setReference(commandPosition, ControlType.kSmartMotion);
  }
}

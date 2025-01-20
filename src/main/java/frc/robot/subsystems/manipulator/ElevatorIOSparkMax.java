package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevator = new SparkMax(0, MotorType.kBrushless); // TEMP ID
  private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private final RelativeEncoder elevatorEncoder = elevator.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(0);

  public ElevatorIOSparkMax() {
    elevatorConfig.closedLoop.maxMotion.maxAcceleration(1200); // placeholder
    elevatorConfig.smartCurrentLimit(Constants.CurrentLimits.NEO);

    elevator.configure(
        elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.height = elevatorEncoder.getPosition();
    inputs.velocity = elevatorEncoder.getVelocity();

    inputs.appliedOutput = elevator.getAppliedOutput();
    inputs.temperature = elevator.getMotorTemperature();
    inputs.current = elevator.getOutputCurrent();
    inputs.limitSwitch = limitSwitch.get();
  }

  public void setPosition(double commandPosition, double arbFF) {
    elevator
        .getClosedLoopController()
        .setReference(
            commandPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFF);
  }
}

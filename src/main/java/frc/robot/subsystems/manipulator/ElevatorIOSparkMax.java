package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkFlex elevator;
  private final SparkMaxConfig elevatorConfig;
  private final RelativeEncoder elevatorEncoder;
  private final DigitalInput limitSwitch;

  public ElevatorIOSparkMax(int id, int limitSwitchID) {
    elevator = new SparkFlex(id, MotorType.kBrushless);
    elevatorConfig = new SparkMaxConfig();
    elevatorEncoder = elevator.getEncoder();
    limitSwitch = new DigitalInput(limitSwitchID);

    elevatorConfig.closedLoop.smartMotion.maxAcceleration(1200).maxVelocity(6700); // placeholder
    elevatorConfig.closedLoop.velocityFF(1.0/6700.0);
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
            commandPosition, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0, arbFF);
  }
}

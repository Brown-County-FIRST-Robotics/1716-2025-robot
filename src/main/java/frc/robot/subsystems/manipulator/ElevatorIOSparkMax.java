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
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkFlex elevator_primary;
  private final SparkFlex elevator_follower;

  public static final double scle = 0.0254 * 5.5 * 2.0 / 15.0; // m/Rotations // travel=29*0.0254
  private final RelativeEncoder elevatorEncoder;
  private final DigitalInput limitSwitch;

  public ElevatorIOSparkMax(int id, int follower_id, int limit_switch_port) {
    elevator_primary = new SparkFlex(id, MotorType.kBrushless);
    elevator_follower = new SparkFlex(follower_id, MotorType.kBrushless);
    limitSwitch = new DigitalInput(limit_switch_port);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorEncoder = elevator_primary.getEncoder();

    elevatorConfig
        .closedLoop
        .smartMotion
        .maxAcceleration(6100)
        .maxVelocity(1000)
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
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    elevator_primary.configure(
        elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig follower = new SparkMaxConfig();
    follower.follow(elevator_primary, true);
    follower.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    elevator_follower.configure(
        follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder.setPosition(0.0);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = elevatorEncoder.getPosition() * scle;
    inputs.velocity = elevatorEncoder.getVelocity() * scle / 60.0;

    inputs.appliedOutput = elevator_primary.getAppliedOutput();
    inputs.temperature_1 = elevator_primary.getMotorTemperature();
    inputs.temperature_2 = elevator_follower.getMotorTemperature();
    inputs.current = elevator_primary.getOutputCurrent();
    inputs.limitSwitch = !limitSwitch.get();
  }

  public void setPosition(double commandPosition, double arbFF) {
    elevator_primary
        .getClosedLoopController()
        .setReference(commandPosition / scle, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0, 0.2);
  }
}

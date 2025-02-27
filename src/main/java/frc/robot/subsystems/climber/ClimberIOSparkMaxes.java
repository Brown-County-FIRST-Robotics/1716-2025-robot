package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ClimberIOSparkMaxes implements ClimberIO {
  private final SparkMax climber;
  private final SparkLimitSwitch limitSwitch;
  private final SparkMaxConfig climberConfig;

  public ClimberIOSparkMaxes(int motorID, int limitSwitchID) {
    climber = new SparkMax(motorID, MotorType.kBrushless);
    limitSwitch = climber.getForwardLimitSwitch();
    climberConfig = new SparkMaxConfig();

    climberConfig.closedLoop.maxMotion.maxAcceleration(
        1200); // placeholder, will be replaced with actual acceleration
    climberConfig.smartCurrentLimit(
        Constants.CurrentLimits.NEO); // sets the limits based on the NEO motors
    climberConfig.idleMode(IdleMode.kBrake);

    climber
        .configure( // persist mode keeps the last data value even after the robot is shut off, in
            // case of power problems
            climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(
      ClimberIOInputs inputs) { // tells the rio to get specific info & stores it in doubles
    inputs.position = climber.getEncoder().getPosition();
    inputs.velocity = climber.getEncoder().getVelocity();
    inputs.temperature = climber.getMotorTemperature();
    inputs.current = climber.getOutputCurrent();
    inputs.appliedOutput = climber.getAppliedOutput();

    inputs.limitSwitch = limitSwitch.isPressed();
  }

  @Override
  public void setPosition(double position) {
    climber.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void setVelocity(double velocity) {
    climber.set(velocity);
  }
}

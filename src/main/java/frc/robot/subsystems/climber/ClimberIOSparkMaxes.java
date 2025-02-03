package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ClimberIOSparkMaxes implements ClimberIO {
  private final SparkMax climberLeft; // temporary ID
  private final SparkMax climberRight;
  private final SparkMaxConfig climberConfig = new SparkMaxConfig();
  // temporary ID

  // Custom methods go here:
  @Override
  public void setVelocities(
      double velocityLeft, double velocityRight) { // sets the motor control type to velocity
    climberLeft
        .getClosedLoopController()
        .setReference(velocityLeft, ControlType.kMAXMotionVelocityControl);
    climberRight
        .getClosedLoopController()
        .setReference(velocityRight, ControlType.kMAXMotionVelocityControl);
  }

  public ClimberIOSparkMaxes(int leftID, int rightID) {
    climberLeft = new SparkMax(leftID, MotorType.kBrushless); // declares motor type & sets ID
    climberRight = new SparkMax(rightID, MotorType.kBrushless);

    climberConfig.closedLoop.maxMotion.maxAcceleration(
        1200); // placeholder, will be replaced with actual acceleration
    climberConfig.smartCurrentLimit(
        Constants.CurrentLimits.NEO); // sets the limits based on the NEO motors

    climberLeft
        .configure( // persist mode keeps the last data value even after the robot is shut off, in
            // case of power problems
            climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberRight.configure(//resets motor config & then sets to required values
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(
      ClimberIOInputs inputs) { // tells the rio to get specific info & stores it in doubles
    inputs.positions =
        new double[] {
          climberLeft.getEncoder().getPosition(), climberRight.getEncoder().getPosition()
        };
    inputs.velocities =
        new double[] {
          climberLeft.getEncoder().getVelocity(), climberRight.getEncoder().getVelocity()
        };
    inputs.temperatures =
        new double[] {climberLeft.getMotorTemperature(), climberRight.getMotorTemperature()};
    inputs.currents =
        new double[] {climberLeft.getOutputCurrent(), climberRight.getOutputCurrent()};
    inputs.appliedOutputs =
        new double[] {climberLeft.getAppliedOutput(), climberRight.getAppliedOutput()};
  }
}

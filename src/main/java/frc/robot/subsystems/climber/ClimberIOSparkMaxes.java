package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimberIOSparkMaxes implements ClimberIO{
    private final SparkMax climberLeft; // temporary ID
    private final SparkMax climberRight;
      private final SparkMaxConfig climberConfig = new SparkMaxConfig();
 // temporary ID


    //Custom methods go here:
    @Override
    public void setVelocities(double velocityLeft, double velocityRight) {
        climberLeft.getClosedLoopController().setReference(velocityLeft, ControlType.kMAXMotionVelocityControl);
        climberRight.getClosedLoopController().setReference(velocityRight, ControlType.kMAXMotionVelocityControl);

    }
    public ClimberIOSparkMaxes(int leftID, int rightID){
        climberLeft = new SparkMax( leftID, MotorType.kBrushless );
        climberRight = new SparkMax( rightID, MotorType.kBrushless );

        climberConfig.closedLoop.maxMotion.maxAcceleration(1200); // placeholder
    climberConfig.smartCurrentLimit(Constants.CurrentLimits.NEO);

    climberLeft.configure(
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberRight.configure(
            climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.positions = new double[]{climberLeft.getEncoder().getPosition(), climberRight.getEncoder().getPosition()};
        inputs.velocities = new double[]{climberLeft.getEncoder().getVelocity(), climberRight.getEncoder().getVelocity()};
        inputs.temperatures = new double[]{climberLeft.getMotorTemperature(), climberRight.getMotorTemperature()};
        inputs.currents = new double[]{climberLeft.getOutputCurrent(), climberRight.getOutputCurrent()};
        inputs.appliedOutputs = new double[]{climberLeft.getAppliedOutput(), climberRight.getAppliedOutput()};
    }
}

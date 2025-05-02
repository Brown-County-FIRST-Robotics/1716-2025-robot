package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimberIOSparkMaxes implements ClimberIO {
  private final SparkMax climber;
  private final Servo servo;

  public ClimberIOSparkMaxes(int motorID, int servoID) {
    climber = new SparkMax(motorID, MotorType.kBrushless);
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.closedLoop.velocityFF(1.0 / 6500.0).p(3.0 / 6500.0);
    climberConfig.closedLoop.smartMotion.maxAcceleration(1200).maxVelocity(5000);
    climberConfig.smartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);
    climberConfig.idleMode(IdleMode.kCoast);

    climber.configure( // persist mode keeps the config in case of a brownout
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    servo = new Servo(servoID);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = climber.getEncoder().getPosition();
    inputs.velocity = climber.getEncoder().getVelocity();
    inputs.temperature = climber.getMotorTemperature();
    inputs.current = climber.getOutputCurrent();
    inputs.appliedOutput = climber.getAppliedOutput();

    inputs.servoPosition = servo.getAngle();
  }

  @Override
  public void setPosition(double position) {
    climber.getClosedLoopController().setReference(position, ControlType.kSmartMotion);
  }

  @Override
  public void setSpeed(double speed) {
    climber.set(speed);
  }

  @Override
  public void setServo(double position) {
    servo.setAngle(position);
  }
}

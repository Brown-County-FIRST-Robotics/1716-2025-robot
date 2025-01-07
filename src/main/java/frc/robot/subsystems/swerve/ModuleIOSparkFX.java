package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** IO layer for a SDS MK4i L2 swerve module using a Falcon 500 as thrust, and a Neo as steering */
public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 6.75;
  private static final double STEER_FREE_RPM = 5676.0;
  private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  private final CANSparkMax steer;
  private final SparkAnalogSensor analogEncoder;
  private final RelativeEncoder relativeEncoder;
  private final SparkPIDController pid;
  private final TalonFX thrust;
  final StatusSignal<Double> velSignal;
  final StatusSignal<Double> posSignal;
  final StatusSignal<Double> errSignal;
  final StatusSignal<Double> tempSignal;
  final StatusSignal<Double> outputSignal;
  double offset;

  final String name;
  final LoggedTunableNumber thrustP =
      new LoggedTunableNumber("Thrust P", 12.0 * 3.0 / (6380.0 / 60.0));
  final LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  final LoggedTunableNumber thrustKV = new LoggedTunableNumber("Thrust KV", 12.0 * 60.0 / 6380.0);
  final LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 1.0 / STEER_FREE_RPM);
  final LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  final LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  final LoggedTunableNumber steerKV =
      new LoggedTunableNumber("Steer KV", 0.7 * STEER_GEAR_RATIO / STEER_FREE_RPM);
  final LoggedTunableNumber offsetTun;
  double off;

  /**
   * Makes a new instance using CAN IDs
   *
   * @param thrustID Thrust motor CAN ID
   * @param steerID Steer motor controller CAN ID
   * @param name The name of the module (e.g. "FL", "BR")
   */
  public ModuleIOSparkFX(int thrustID, int steerID, String name) {
    this.name = name;
    thrust = new TalonFX(thrustID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    thrust.getConfigurator().refresh(config);
    config.Audio.BeepOnConfig = false;
    config.Audio.BeepOnBoot = false;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = -1;
    config.Audio.AllowMusicDurDisable = true;
    config.Slot0.kV = thrustKV.get();
    config.Slot0.kP = thrustP.get();
    config.Slot0.kI = thrustI.get();
    config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
    offsetTun = new LoggedTunableNumber(name + "_offset");
    if (thrustID == 20) {
      off = 0.812; // BR
    } else if (thrustID == 21) {
      off = 0.01; // BL
    } else if (thrustID == 22) {
      off = -0.03; // FL
    } else if (thrustID == 23) {
      off = 0.45; // FR
    } else if (thrustID == 24) {
      off = 0.069; // FR (backup)
    }
    offsetTun.initDefault(off);
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    thrust.getConfigurator().apply(config);
    velSignal = thrust.getRotorVelocity();
    posSignal = thrust.getRotorPosition();
    errSignal = thrust.getClosedLoopError();
    tempSignal = thrust.getDeviceTemp();
    outputSignal = thrust.getClosedLoopOutput();
    velSignal.setUpdateFrequency(50.0);
    posSignal.setUpdateFrequency(50.0);
    errSignal.setUpdateFrequency(50.0);
    outputSignal.setUpdateFrequency(50.0);
    tempSignal.setUpdateFrequency(20.0);
    thrust.optimizeBusUtilization();
    steer = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
    steer.restoreFactoryDefaults();
    steer.setIdleMode(CANSparkBase.IdleMode.kBrake);
    pid = steer.getPIDController();
    analogEncoder = steer.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    relativeEncoder = steer.getEncoder();
    relativeEncoder.setPositionConversionFactor(1.0 / STEER_GEAR_RATIO);
    analogEncoder.setPositionConversionFactor(1 / 3.33);
    analogEncoder.setInverted(true);
    steer.setInverted(true);
    pid.setFeedbackDevice(relativeEncoder);

    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(STEER_FREE_RPM / STEER_GEAR_RATIO, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(10 * STEER_FREE_RPM / STEER_GEAR_RATIO, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.002, 0);
    steer.setSmartCurrentLimit(Constants.CurrentLimits.NEO);

    steerKV.attach(pid::setFF);
    steerP.attach(pid::setP);
    steerI.attach(pid::setI);
    steerD.attach(pid::setD);
    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);

    steer.burnFlash();
    Logger.recordOutput("Firmware/" + name + "_Steer", steer.getFirmwareString());
    Logger.recordOutput("Firmware/" + name + "_Thrust", thrust.getVersion().getValue());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);
    inputs.absSensorAngle = analogEncoder.getPosition();
    inputs.absSensorOmega = analogEncoder.getVelocity();
    inputs.relativeSensorAngle = relativeEncoder.getPosition();
    inputs.relativeSensorOmega = relativeEncoder.getVelocity() / 60.0;
    inputs.thrustVel = velSignal.getValue() * THRUST_DISTANCE_PER_TICK;
    inputs.thrustPos = posSignal.getValue() * THRUST_DISTANCE_PER_TICK;
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = errSignal.getValue();
    inputs.thrustTempC = tempSignal.getValue();
    inputs.offset = offsetTun.get();
    inputs.thrustOutput = outputSignal.getValue();
  }

  @Override
  public void setCmdState(double ang, double vel) {
    thrust.setControl(new VelocityVoltage(vel / THRUST_DISTANCE_PER_TICK));
    pid.setReference(ang, CANSparkMax.ControlType.kSmartMotion);
  }
}

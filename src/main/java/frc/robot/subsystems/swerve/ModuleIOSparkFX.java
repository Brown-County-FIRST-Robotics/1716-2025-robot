package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** IO layer for a SDS MK4i L2 swerve module using a Falcon 500 as thrust, and a Neo as steering */
public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 6.75;
  private static final double STEER_FREE_RPM = 5676.0;
  private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  private final SparkMax steer;
  private final SparkAbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController pid;
  private final TalonFX thrust;
  final StatusSignal<AngularVelocity> velSignal;
  final StatusSignal<Angle> posSignal;
  final StatusSignal<Double> errSignal;
  final StatusSignal<Temperature> tempSignal;
  final StatusSignal<Double> outputSignal;
  double offset;

  final String name;
  final LoggedTunableNumber thrustP =
      new LoggedTunableNumber("Thrust P", 12.0 * 3.0 / (6380.0 / 60.0));
  final LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  final LoggedTunableNumber thrustKV = new LoggedTunableNumber("Thrust KV", 12.0 / 6380.0);
  final LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 0);
  final LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  final LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  final LoggedTunableNumber steerKV = new LoggedTunableNumber("Steer KV", 1.0 / STEER_FREE_RPM);
  final LoggedTunableNumber offsetTun;
  double off;
  SparkMaxConfig z = new SparkMaxConfig();

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
    config.Slot0.kV = 12.0 * 60.0 / 6380.0; // thrustKV.get();
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
      off = 0.348; // FR? ID23 NEW WHEEL
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
    steer = new SparkMax(steerID, SparkLowLevel.MotorType.kBrushless);
    var adfs =
        new SparkMaxConfig()
            .smartCurrentLimit(Constants.CurrentLimits.NEO)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    var closedloopconf = adfs.closedLoop;
    var relencoderconf = adfs.encoder.positionConversionFactor(1.0 / STEER_GEAR_RATIO);
    var analogconf = adfs.analogSensor.inverted(true);
    // TODO: FIX
    // This probably doesn't work, but we need a more permanant solution later
    closedloopconf =
        closedloopconf
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .outputRange(-1, 1)
            .pidf(steerP.get(), steerI.get(), steerD.get(), steerKV.get());
    var smartconf = closedloopconf.maxMotion;
    smartconf =
        smartconf
            .maxVelocity(STEER_FREE_RPM / STEER_GEAR_RATIO)
            .maxAcceleration(10 * STEER_FREE_RPM / STEER_GEAR_RATIO)
            .allowedClosedLoopError(0.002);
    adfs = adfs.apply(relencoderconf).apply(analogconf).apply(closedloopconf.apply(smartconf));

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    z.encoder.positionConversionFactor(1);
    z.absoluteEncoder.inverted(false);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    steerP.attach(
        (Double p) -> {
          z.closedLoop.p(p);
          steer.configure(z, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    steerI.attach(
        (Double p) -> {
          z.closedLoop.i(p);
          steer.configure(z, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    steerD.attach(
        (Double p) -> {
          z.closedLoop.d(p);
          steer.configure(z, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    z.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0) // steerP.get())
        .i(0) // steerI.get())
        .d(0) // steerD.get())
        .outputRange(-12, 12)
        .velocityFF(1.0 / 5676.0);
    z.closedLoop.smartMotion.maxVelocity(1000).maxAcceleration(1000).allowedClosedLoopError(0.0025);

    z.closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(0.01);
    z.voltageCompensation(12.0);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    steer.configure(z, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // steer.configure(
    //     adfs, SparkBase.ResetMode.kResetSafeParameters,
    // SparkBase.PersistMode.kPersistParameters);

    pid = steer.getClosedLoopController();
    absoluteEncoder = steer.getAbsoluteEncoder();
    relativeEncoder = steer.getEncoder();
    steer.setInverted(true);
    // steer.setInverted(tr[]\
    // ue);

    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);

    Logger.recordOutput("Firmware/" + name + "_Steer", steer.getFirmwareString());
    Logger.recordOutput("Firmware/" + name + "_Thrust", thrust.getVersion().getValue());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);
    inputs.absSensorAngle = absoluteEncoder.getPosition();
    inputs.absSensorOmega = absoluteEncoder.getVelocity();
    inputs.relativeSensorAngle = relativeEncoder.getPosition() * 7.0 / 150.0;
    inputs.relativeSensorOmega = relativeEncoder.getVelocity() / 60.0;
    inputs.thrustVel = velSignal.getValue().in(Units.RotationsPerSecond) * THRUST_DISTANCE_PER_TICK;
    inputs.thrustPos = posSignal.getValue().in(Units.Rotations) * THRUST_DISTANCE_PER_TICK;
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = errSignal.getValue();
    inputs.thrustTempC = tempSignal.getValue().in(Units.Celsius);
    inputs.offset = offsetTun.get();
    inputs.thrustOutput = outputSignal.getValue();
  }

  @Override
  public void setCmdState(double ang, double vel) {
    vel=0.5;
    thrust.setControl(new VelocityVoltage(vel / THRUST_DISTANCE_PER_TICK));
    pid.setReference(ang * 150.0 / 7.0, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0);
    // steer.set(0.2);
    // pid.setReference(0.5, SparkMax.ControlType.kMAXMotionPositionControl);
    // TODO: Use non-decrepeted version
  }
}

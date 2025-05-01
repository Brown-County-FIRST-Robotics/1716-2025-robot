package frc.robot.subsystems.mecanum;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** The mecanum IO implementation for 4 SPARKMAX motor controllers */
@Deprecated(forRemoval = true, since = "20250501")
public class MecanumIOSpark implements MecanumIO {
  static final double EFFECTIVE_WHEEL_DIAMETER = 0.05411255411255412;
  final SparkMax fl;
  final SparkMax fr;
  final SparkMax bl;
  final SparkMax br;
  final RelativeEncoder flEncoder;
  final RelativeEncoder frEncoder;
  final RelativeEncoder blEncoder;
  final RelativeEncoder brEncoder;
  final SparkClosedLoopController flPID;
  final SparkClosedLoopController frPID;
  final SparkClosedLoopController blPID;
  final SparkClosedLoopController brPID;
  final LoggedTunableNumber ffTuner = new LoggedTunableNumber("Mecanum FF", 1.0 / 6500);
  final LoggedTunableNumber pTuner = new LoggedTunableNumber("Mecanum P", 0);
  final LoggedTunableNumber iTuner = new LoggedTunableNumber("Mecanum I", 0);
  final LoggedTunableNumber dTuner = new LoggedTunableNumber("Mecanum D", 0);

  /**
   * Constructs a <code>MecanumIOSpark</code> from CAN IDs
   *
   * @param flID Front left CAN ID
   * @param frID Front right CAN ID
   * @param blID Back left CAN ID
   * @param brID Back right CAN ID
   */
  public MecanumIOSpark(int flID, int frID, int blID, int brID) {
    fl = new SparkMax(flID, SparkLowLevel.MotorType.kBrushless);
    flEncoder = fl.getEncoder();
    flPID = fl.getClosedLoopController();
    fr = new SparkMax(frID, SparkLowLevel.MotorType.kBrushless);
    frEncoder = fr.getEncoder();
    frPID = fr.getClosedLoopController();
    bl = new SparkMax(blID, SparkLowLevel.MotorType.kBrushless);
    blEncoder = bl.getEncoder();
    blPID = bl.getClosedLoopController();
    br = new SparkMax(brID, SparkLowLevel.MotorType.kBrushless);
    brEncoder = br.getEncoder();
    brPID = br.getClosedLoopController();

    var adfs =
        new SparkMaxConfig()
            .smartCurrentLimit(Constants.CurrentLimits.NEO)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    var closedloopconf = adfs.closedLoop;
    // TODO: FIX
    // This probably doesn't work, but we need a more permanant solution later
    closedloopconf =
        closedloopconf
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1)
            .pidf(0, 0, 0, 0); // TODO: FIX
    var smartconf = closedloopconf.maxMotion;

    // TODO: verify this
    smartconf = smartconf.maxVelocity(6500).maxAcceleration(65000).allowedClosedLoopError(0.002);
    adfs = adfs.apply(closedloopconf.apply(smartconf));
    fl.configure(
        adfs, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    fr.configure(
        adfs, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    bl.configure(
        adfs, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    br.configure(
        adfs, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    Logger.recordOutput("Firmware/FLController", fl.getFirmwareString());
    Logger.recordOutput("Firmware/FRController", fr.getFirmwareString());
    Logger.recordOutput("Firmware/BLController", bl.getFirmwareString());
    Logger.recordOutput("Firmware/BRController", br.getFirmwareString());
  }

  @Override
  public void setSpeeds(MecanumDriveWheelSpeeds cmd) {
    flPID.setReference(
        60 * cmd.frontLeftMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        SparkBase.ControlType.kVelocity);
    frPID.setReference(
        60 * cmd.frontRightMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        SparkBase.ControlType.kVelocity);
    blPID.setReference(
        60 * cmd.rearLeftMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        SparkBase.ControlType.kVelocity);
    brPID.setReference(
        60 * cmd.rearRightMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        SparkBase.ControlType.kVelocity);
  }

  @Override
  public void updateInputs(MecanumIOInputs inputs) {
    inputs.flTemp = fl.getMotorTemperature();
    inputs.frTemp = fr.getMotorTemperature();
    inputs.blTemp = bl.getMotorTemperature();
    inputs.brTemp = br.getMotorTemperature();
    inputs.pos =
        new MecanumDriveWheelPositions(
            flEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            frEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            blEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            brEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER);
    inputs.vel =
        new MecanumDriveWheelSpeeds(
            flEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            frEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            blEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            brEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0);
    inputs.flOut = fl.getAppliedOutput();
    inputs.frOut = fr.getAppliedOutput();
    inputs.blOut = bl.getAppliedOutput();
    inputs.brOut = br.getAppliedOutput();
  }
}

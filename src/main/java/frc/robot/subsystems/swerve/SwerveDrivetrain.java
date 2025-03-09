package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utils.Overrides;
import frc.robot.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;

/** The swerve drivetrain subsystem */
public class SwerveDrivetrain implements Drivetrain {
  private static final double D = 21 * 0.0254; // TODO: Rename this
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(D / 2, D / 2),
          new Translation2d(D / 2, -D / 2),
          new Translation2d(-D / 2, D / 2),
          new Translation2d(-D / 2, -D / 2));
  private static final double MAX_WHEEL_SPEED = 5.0;
  final Module fl;
  final Module fr;
  final Module bl;
  final Module br;

  Rotation3d lastIMU;
  SwerveModulePosition[] lastPositions;
  final PoseEstimator poseEstimator;

  final IMUIO imu;
  final IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();

  private SwerveModulePosition[] getPositions() {

    return new SwerveModulePosition[] {
      fl.getChassisRelativePosition(),
      fr.getChassisRelativePosition(),
      bl.getChassisRelativePosition(),
      br.getChassisRelativePosition()
    };
  }

  /**
   * Creates a SwerveDrivetrain from IO
   *
   * @param fl Front left module IO
   * @param fr Front right module IO
   * @param bl Back left module IO
   * @param br Back right module IO
   * @param imu IMU IO
   */
  public SwerveDrivetrain(Module fl, Module fr, Module bl, Module br, IMUIO imu) {
    this.imu = imu;
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    poseEstimator = new PoseEstimator();
    //    poseEstimator.setPose(Constants.INIT_POSE);
    lastIMU = getGyro();
    lastPositions = getPositions();
  }

  @Override
  public void periodic() {
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    fl.periodic();
    fr.periodic();
    bl.periodic();
    br.periodic();

    Logger.recordOutput("Drive/RealStates", getWheelSpeeds());
    Twist2d odoTwist = KINEMATICS.toTwist2d(lastPositions, getPositions());
    if (!Overrides.disableIMU.get()) {
      odoTwist = new Twist2d(odoTwist.dx, odoTwist.dy, -getGyro().minus(lastIMU).getX());
    }
    lastPositions = getPositions();
    lastIMU = getGyro();
    Logger.recordOutput("Drive/Pose", getPosition());

    checkForYawReset();
  }

  private void checkForYawReset() {
    if (Overrides.resetYaw.get()) {
      poseEstimator.setPose(
          new Pose2d(getPosition().getTranslation(), Constants.INIT_POSE.getRotation()));
      Overrides.resetYaw.set(false);
    }
  }

  private SwerveModuleState[] getWheelSpeeds() {
    return new SwerveModuleState[] {
      fl.getChassisRelativeState(),
      fr.getChassisRelativeState(),
      bl.getChassisRelativeState(),
      br.getChassisRelativeState()
    };
  }

  @Override
  public Pose2d getPosition() {
    return poseEstimator.getPose();
  }

  private void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
    Logger.recordOutput("Drive/CmdStates", states);
    fl.setState(states[0]);
    fr.setState(states[1]);
    bl.setState(states[2]);
    br.setState(states[3]);
  }

  @Override
  public void setPosition(Pose2d pos) {
    poseEstimator.setPose(pos);
  }

  @Override
  public void addVisionUpdate(Pose2d newPose, Vector<N3> stdDevs, double timestamp) {}

  @Override
  public void humanDrive(ChassisSpeeds cmd) {
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(cmd);
    setModuleStates(states);
  }

  @Override
  public Rotation3d getGyro() {
    return imuInputs.rotation;
  }

  @Override
  public double[] getAcceleration() {
    return new double[] {imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }

  @Override
  public void lockWheels() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
  }

  @Override
  public PoseEstimator getPE() {
    return poseEstimator;
  }

  @Override
  public void followTrajectory(SwerveSample sample) {
    humanDrive(
        new ChassisSpeeds(
            sample.vx, sample.vy, sample.omega)); // TODO: probably use PID or something
  }
}

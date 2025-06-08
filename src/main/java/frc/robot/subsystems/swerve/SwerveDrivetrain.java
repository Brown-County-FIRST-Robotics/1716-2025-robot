package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.*;
import frc.robot.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;

/** The swerve drivetrain subsystem */
public class SwerveDrivetrain implements Drivetrain {
  private static final double WHEEL_SPACING = 21 * 0.0254;
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_SPACING / 2, WHEEL_SPACING / 2),
          new Translation2d(WHEEL_SPACING / 2, -WHEEL_SPACING / 2),
          new Translation2d(-WHEEL_SPACING / 2, WHEEL_SPACING / 2),
          new Translation2d(-WHEEL_SPACING / 2, -WHEEL_SPACING / 2));
  private static final double MAX_WHEEL_SPEED = 5.0;
  final Module fl;
  final Module fr;
  final Module bl;
  final Module br;

  final PoseEstimator poseEstimator;

  /**
   * Creates a SwerveDrivetrain from IO
   *
   * @param fl Front left module IO
   * @param fr Front right module IO
   * @param bl Back left module IO
   * @param br Back right module IO
   */
  public SwerveDrivetrain(Module fl, Module fr, Module bl, Module br) {
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    poseEstimator = new PoseEstimator();
    //    poseEstimator.setPose(Constants.INIT_POSE);
  }

  @Override
  public void periodic() {
    fl.periodic();
    fr.periodic();
    bl.periodic();
    br.periodic();

    Logger.recordOutput("Drive/RealStates", getWheelSpeeds());
    Logger.recordOutput("Drive/Pose", getPosition());
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

  PIDController xPid = new PIDController(1.0, 0.0, 0.0);
  PIDController yPid = new PIDController(1.0, 0.0, 0.0);
  PIDController thPid = new PIDController(1.0, 0.0, 0.0);

  @Override
  public void followTrajectory(SwerveSample sample) {
    humanDrive(
        new ChassisSpeeds(
            sample.vx, sample.vy, sample.omega)); // TODO: probably use PID or something
  }
}

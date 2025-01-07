package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A command that follows a trajectory */
public class HolonomicTrajectoryFollower extends Command {
  private static final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(10, 10);
  /** The maximum allowed rotation error */
  public static final LoggedTunableNumber allowedErr =
      new LoggedTunableNumber("Rotation Allowed Err", 3);

  private static final LoggedTunableNumber replanErr =
      new LoggedTunableNumber("Replanning threshold", 0.1);

  /**
   * Returns the command velocity to get to a rotational goal
   *
   * @param cmdRotation The commanded rotation
   * @param currentRotation The current rotation
   * @param currentVelocity The current angular velocity
   * @return The command angular velocity
   */
  public static double getExt(
      Rotation2d cmdRotation, Rotation2d currentRotation, double currentVelocity) {
    double goal = cmdRotation.getRadians();
    if (Math.abs(currentRotation.getRadians() - goal) > Math.PI) {
      goal -= 2 * Math.PI;
      if (Math.abs(currentRotation.getRadians() - goal) > Math.PI) {
        goal += 4 * Math.PI;
      }
    }
    var tp = new TrapezoidProfile(constraints);
    double fvel =
        tp.calculate(
                0.02,
                new TrapezoidProfile.State(currentRotation.getRadians(), currentVelocity),
                new TrapezoidProfile.State(goal, 0))
            .velocity;

    return tp.totalTime() > 0.02 ? fvel : 0;
  }

  final Drivetrain drivetrain;
  final Supplier<Trajectory> trajectorySupplier;
  Trajectory activeTrajectory;
  final Timer timer = new Timer();
  Optional<Rotation2d> customRotation = Optional.empty();

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  public HolonomicTrajectoryFollower(
      Drivetrain drive, Supplier<Trajectory> trajectorySupplier, Rotation2d targetRotation) {
    this(drive, trajectorySupplier);
    setCustomRotation(Optional.of(targetRotation));
  }

  public HolonomicTrajectoryFollower(Drivetrain drive, Supplier<Trajectory> trajectorySupplier) {
    this.drivetrain = drive;
    this.trajectorySupplier = trajectorySupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    activeTrajectory = trajectorySupplier.get();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (activeTrajectory
            .sample(timer.get())
            .poseMeters
            .getTranslation()
            .getDistance(drivetrain.getPosition().getTranslation())
        > replanErr.get()) {
      activeTrajectory = trajectorySupplier.get();
      timer.reset();
      timer.start();
    }
    var state = activeTrajectory.sample(timer.get());
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getCos(),
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getSin(),
            customRotation
                .map(
                    rotation2d ->
                        getExt(
                            rotation2d,
                            drivetrain.getPosition().getRotation(),
                            drivetrain.getVelocity().omegaRadiansPerSecond))
                .orElse(0.0));
    var discreteSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getPosition().getRotation());
    drivetrain.humanDrive(discreteSpeeds);
    Logger.recordOutput("Follower/CurrentTrajectory", activeTrajectory);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.humanDrive(new ChassisSpeeds());
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(activeTrajectory.getTotalTimeSeconds())
        && (customRotation
            .filter(
                rotation2d ->
                    (Math.abs(rotation2d.minus(drivetrain.getPosition().getRotation()).getDegrees())
                        < allowedErr.get()))
            .isEmpty());
  }

  public Translation2d getPoseAtTime(double t) {
    return activeTrajectory.sample(timer.get() + t).poseMeters.getTranslation();
  }

  public Translation2d getSpeedAtTime(double t) {
    return new Translation2d(
        activeTrajectory.sample(timer.get() + t).velocityMetersPerSecond,
        activeTrajectory.sample(timer.get() + t).poseMeters.getRotation());
  }
}

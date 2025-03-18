package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class GoToPoseQM extends Command {
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 7.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 7.0), 0.02);
  final Drivetrain drivetrain;
  final Pose2d target;

  public GoToPoseQM(Drivetrain drivetrain, Pose2d target) {
    this.drivetrain = drivetrain;
    this.target = target;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.humanDrive(new ChassisSpeeds());
  }

  @Override
  public void execute() {
    Pose2d thrustDirection = target.relativeTo(drivetrain.getPosition());
    double thrustTranslationDistance = target.getTranslation().getNorm();
    double thrustTranslationPower = driveController.calculate(thrustTranslationDistance, 0);
    Translation2d thrustVec =
        thrustDirection
            .getTranslation()
            .div(thrustDirection.getTranslation().getNorm())
            .times(thrustTranslationPower);
    Logger.recordOutput("sadf", thrustDirection);
    double rotationThrust =
        thetaController.calculate(thrustDirection.getRotation().getRotations(), 0);
    System.out.println(rotationThrust);
    drivetrain.humanDrive(new ChassisSpeeds(-thrustVec.getX(), -thrustVec.getY(), rotationThrust));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.humanDrive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    Twist2d err = drivetrain.getPosition().log(target);
    return Math.abs(err.dtheta) < 0.1 && (err.dx * err.dx + err.dy * err.dy) < 0.2;
  }
}

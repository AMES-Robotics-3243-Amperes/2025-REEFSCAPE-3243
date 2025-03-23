package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.splines.Path;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.swerve.control.DriveWithSpeeds;

public class CommandSwerveFollowSpline extends Command {
  private SubsystemSwerveDrivetrain drivetrain;
  private Path path;
  private Rotation2d minimumRotationTolerance;

  private PIDController xController;
  private PIDController yController;
  private ProfiledPIDController thetaController;

  public CommandSwerveFollowSpline(SubsystemSwerveDrivetrain drivetrain,
      Path path,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.path = path;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    thetaController.reset(MathUtil.angleModulus(path.getCurrentPosition().getRotation().getRadians()));
    xController.reset();
    yController.reset();

    path.initialize();
    this.minimumRotationTolerance = path.getMinimumRotationTolerance();
  }

  @Override
  public void execute() {
    Translation2d robotPosition = path.getCurrentPosition().getTranslation();
    Rotation2d robotRotation = path.getCurrentPosition().getRotation();
    Translation2d goal = path.getGoalPosition();

    double xValue = xController.calculate(robotPosition.getX() - goal.getX());
    double yValue = yController.calculate(robotPosition.getY() - goal.getY());
    Translation2d pidAdjustment = new Translation2d(xValue, yValue);

    double rotationSpeed = 0;
    if (path.getDesiredRotation().isPresent()) {
      rotationSpeed = thetaController.calculate(MathUtil.angleModulus(robotRotation.getRadians()),
          MathUtil.angleModulus(path.getDesiredRotation().get().getRadians()));

      if (Math.abs(MathUtil.angleModulus(
          path.getDesiredRotation().get().minus(robotRotation).getRadians())) > minimumRotationTolerance.getRadians())
        rotationSpeed += FollowConstants.staticThetaVelocity * Math.signum(rotationSpeed);
    }

    drivetrain
        .setControlRequest(DriveWithSpeeds.newRequest(path.getDesiredVelocity().plus(pidAdjustment), rotationSpeed)
            .fieldRelative(true).limitAccelaration(false).limitSpeed(false));

    path.advance();
  }

  @Override
  public boolean isFinished() {
    return path.isComplete();
  }
}

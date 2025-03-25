package frc.robot.commands.automatics;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.DataManager;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.tasks.FinishBeforeTask;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

public class PositionUtils {
  public static class PositionUtilsConstants {
    // the distance between the edge of the robot chasis and the apriltag
    // in the given setpoins. remember to account for bumpers.
    public static double offsetFromTag = Units.inchesToMeters(3);

    public static double distanceBetweenReefs = Units.inchesToMeters(12.9);
  }

  public static Pose2d getNearestReefTagPose() {
    Pose2d robotPosition = DataManager.instance().robotPosition.get();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();

    boolean isBlueAlliance = currentAlliance.isPresent()
        ? currentAlliance.get() == Alliance.Blue
        : true;

    if (isBlueAlliance) {
      return robotPosition.nearest(List.of(FieldConstants.blueReef1.toPose2d(), FieldConstants.blueReef2.toPose2d(),
          FieldConstants.blueReef3.toPose2d(), FieldConstants.blueReef4.toPose2d(), FieldConstants.blueReef5.toPose2d(),
          FieldConstants.blueReef6.toPose2d()));
    } else {
      return robotPosition.nearest(List.of(FieldConstants.redReef1.toPose2d(), FieldConstants.redReef2.toPose2d(),
          FieldConstants.redReef3.toPose2d(), FieldConstants.redReef4.toPose2d(), FieldConstants.redReef5.toPose2d(),
          FieldConstants.redReef6.toPose2d()));
    }
  }

  public static Pose2d getReefScoringPosition(Pose2d reefAprilTagPose, boolean scoreLeft) {
    // "left" in this case is from the perspective of facing the aprilag
    double horizontalOffset = scoreLeft ? -PositionUtilsConstants.distanceBetweenReefs / 2.0
        : PositionUtilsConstants.distanceBetweenReefs / 2.0;
    double verticalOffset = ChassisKinematics.kRobotLength / 2.0 + PositionUtilsConstants.offsetFromTag;

    Transform2d tagToScoringPosition = new Transform2d(verticalOffset, horizontalOffset, Rotation2d.k180deg);
    return reefAprilTagPose.transformBy(tagToScoringPosition);
  }

  public static Pose2d getNearestReefScoringPosition(boolean scoreLeft) {
    return getReefScoringPosition(getNearestReefTagPose(), scoreLeft);
  }

  public static Pose2d getNearestIntakePosition() {
    Pose2d robotPosition = DataManager.instance().robotPosition.get();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();

    boolean isBlueAlliance = currentAlliance.isPresent()
        ? currentAlliance.get() == Alliance.Blue
        : true;

    Pose2d nearestIntakeTag;
    if (isBlueAlliance) {
      nearestIntakeTag = robotPosition.nearest(
          List.of(FieldConstants.blueCoralLoadingBottom.toPose2d(), FieldConstants.blueCoralLoadingTop.toPose2d()));
    } else {
      nearestIntakeTag = robotPosition.nearest(
          List.of(FieldConstants.redCoralLoadingBottom.toPose2d(), FieldConstants.redCoralLoadingTop.toPose2d()));
    }

    double offset = ChassisKinematics.kRobotLength / 2.0 + PositionUtilsConstants.offsetFromTag;
    Transform2d tagToScoringPosition = new Transform2d(offset, 0, Rotation2d.kZero);
    return nearestIntakeTag.transformBy(tagToScoringPosition);
  }

  public static Pose2d getIntakePosition(boolean top) {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();

    boolean isBlueAlliance = currentAlliance.isPresent()
        ? currentAlliance.get() == Alliance.Blue
        : true;

    Pose2d desiredIntakeTag;
    if (isBlueAlliance && top) {
      desiredIntakeTag = FieldConstants.blueCoralLoadingTop.toPose2d();
    } else if (isBlueAlliance && !top) {
      desiredIntakeTag = FieldConstants.blueCoralLoadingBottom.toPose2d();
    } else if (!isBlueAlliance && top) {
      desiredIntakeTag = FieldConstants.redCoralLoadingTop.toPose2d();
    } else {
      desiredIntakeTag = FieldConstants.redCoralLoadingBottom.toPose2d();
    }

    double offset = ChassisKinematics.kRobotLength / 2.0 + PositionUtilsConstants.offsetFromTag;
    Transform2d tagToScoringPosition = new Transform2d(offset, 0, Rotation2d.kZero);
    return desiredIntakeTag.transformBy(tagToScoringPosition);
  }

  public static Command moveToNearestScorePositionCommand(boolean scoreLeft, double elevatorSetpoint,
      SubsystemSwerveDrivetrain drivetrain, SubsystemElevator elevator) {
    return new DeferredCommand(new Supplier<Command>() {
      @Override
      public Command get() {
        Pose2d nearestScoringPosition = getNearestReefScoringPosition(scoreLeft);

        return PathFactory.newFactory()
            .addTask(nearestScoringPosition.getTranslation(),
                new FinishBeforeTask(new ElevatorMoveToPositionCommand(elevator, elevatorSetpoint)))
            .interpolateFromStart(true)
            .interpolator(new LinearInterpolator())
            .buildCommand(drivetrain, FollowConstants.xyController(), FollowConstants.xyController(),
                FollowConstants.thetaController());
      }
    },
        Set.of(drivetrain, elevator));
  }
}

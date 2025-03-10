// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Iterator;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.DataManager.Setpoint;
import frc.robot.DataManager;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.claw.DeployClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.splines.tasks.PerformAtTask;
import frc.robot.subsystems.SubsystemClaw;

/**
 * Scores a coral in the reef automatically
 * 
 * @author Jasper Davidson
 */
public class ScoreIntakeAutoCommandBuilder {
  /** Creates a new ScoreInReefCommand. */
  public static Command scoreIntakeAutoCommand(
      SubsystemSwerveDrivetrain drivetrain, SubsystemClaw diffClaw, SubsystemElevator elevator,
      Setpoint reefPosition, double tagOffset) {
    ProxyCommand command = new ProxyCommand(() -> {
      // Find the closest tag to the robot's current position
      Pose2d currentPose = DataManager.instance().robotPosition.get();
      Translation2d currentPosition = currentPose.getTranslation();
      double minDistance = Double.MAX_VALUE;

      // Initialize to any tag --> It *will* get overran (I hope there's a tag closer
      // than Double.MAX_VALUE...)
      Pose2d closestTag = FieldConstants.blueReef1.toPose2d();

      for (AprilTag tag : FieldConstants.tagList) {
        Pose2d tagPosition = tag.pose.toPose2d();
        double potentialMin = (tagPosition.getTranslation()).getDistance(currentPosition);

        if (potentialMin < minDistance) {
          minDistance = potentialMin;
          closestTag = tagPosition;
        }
      }

      PathFactory pathFactory = PathFactory.newFactory();

      pathFactory
          .interpolateFromStart(true)
          .interpolator(new LinearInterpolator())
          .taskDampen((remainingLength) -> 2 * remainingLength + 0.05);
      moveToPositionTaskBuilder(closestTag, pathFactory, diffClaw, elevator, reefPosition,
          tagOffset);

      return pathFactory.buildCommand(
          drivetrain,
          FollowConstants.xyController(),
          FollowConstants.xyController(),
          FollowConstants.thetaController());
    });
    command.addRequirements(drivetrain, elevator, diffClaw);
    return command;
  }

  public static void moveToPositionTaskBuilder(Pose2d tagPosition, PathFactory pathFactory,
      SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset) {
    double distanceFromTag = 0.55;
    Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, tagOffset), new Rotation2d());
    Translation2d targetPosition = tagPosition.plus(offset).getTranslation();
    Rotation2d targetRotation = tagPosition.getRotation().plus(Rotation2d.fromDegrees(180));

    pathFactory
        .addTask(targetPosition, new FinishByTask(new ElevatorMoveToPositionCommand(elevator, targetSetpoint.height)))
        .addTask(targetPosition, new FinishByTask(new InstantCommand(
            () -> {
              diffClaw.setOutsidePosition(targetSetpoint.angle);
            },
            diffClaw)))
        .addTask(targetPosition,
            new PerformAtTask(targetRotation, new DeployClawCommand(diffClaw, -Setpoints.intakePower)));
  }

  public static Command buildAuto(List<Pose2d> targetPositions, PathFactory pathFactory,
      SubsystemClaw diffClaw, SubsystemElevator elevator, SubsystemSwerveDrivetrain drivetrain,
      List<Setpoint> targetSetpoints) {
    Iterator<Pose2d> positions = targetPositions.iterator();
    Iterator<Setpoint> setpoints = targetSetpoints.iterator();

    while (positions.hasNext() && setpoints.hasNext()) {
      Setpoint setpoint = setpoints.next();
      moveToPositionTaskBuilder(positions.next(), pathFactory, diffClaw, elevator, setpoints.next(),
          setpoint.offset);
    }

    return pathFactory.interpolateFromStart(true).buildCommand(
        drivetrain, FollowConstants.xyController(), FollowConstants.xyController(),
        FollowConstants.thetaController());
  }
}

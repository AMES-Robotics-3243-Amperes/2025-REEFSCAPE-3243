// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.tasks.FinishBeforeTask;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

/**
 * Goes to a loading station and intakes a coral.
 */
public class LoadAutoCommand extends SequentialCommandGroup {
  /** Creates a new LoadAutoCommand. */
  public LoadAutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemElevator elevator, SubsystemEndEffector endEffector, boolean top) {
    super
    (
      new DeferredCommand(
        () -> 
        {
          var loadingPosition = PositionUtils.getIntakePosition(top);
          return PathFactory.newFactory()
            .addTask(loadingPosition.getTranslation(),
              new FinishBeforeTask(Optional.of(loadingPosition.getRotation()), TaskConstants.defaultRotationTolerance,
                TaskConstants.defaultPositionBuffer, new ElevatorMoveToPositionCommand(elevator, ElevatorMoveToPositionCommand.Position.Loading)))
            .interpolateFromStart(true)
            .interpolator(new LinearInterpolator())
            .buildCommand(drivetrain);
        },
        Set.of(drivetrain, elevator)
      ),
      endEffector.intakeCommand()
    );
  }
}

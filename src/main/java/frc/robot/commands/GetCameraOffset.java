// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class GetCameraOffset extends Command {
  private PhotonCamera camera;
  private Transform3d robotToTag;
  private Timer timer = new Timer();

  private Transform3d transformSums = new Transform3d();
  private long transformCount = 0;

  /** Creates a new GetCameraOffset. */
  public GetCameraOffset(PhotonCamera camera, Transform3d robotToTag) {
    this.camera = camera;
    this.robotToTag = robotToTag;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.poseAmbiguity > 0.04) {
        continue;
      }
      
      transformSums = transformSums.plus(target.bestCameraToTarget);
      transformCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Transform3d cameraToTarget = transformSums.div(transformCount);
    Transform3d robotToCamera = robotToTag.plus(cameraToTarget.inverse());

    System.out.println("=====================");
    System.out.println("Camera Offset Results");
    System.out.println("=====================");
    System.out.println("Robot To Camera Rotation:" + robotToCamera.getRotation());
    System.out.println("Robot To Camera Translation:" + robotToCamera.getTranslation());
    System.out.println("Robot To Camera Transform:" + robotToCamera);
    System.out.println("=====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(10);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonvisionConstants;

public class GetCameraOffset extends Command {
  private PhotonCamera camera;
  private Transform3d robotToTag;
  private Timer timer = new Timer();

  private Translation3d translationSums = new Translation3d();
  private Rotation3d cameraToTagRotation = new Rotation3d();
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
    translationSums = new Translation3d();
    cameraToTagRotation = new Rotation3d();
    transformCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      PhotonTrackedTarget target = result.getBestTarget();

      if (target == null) {
        continue;
      }

      if (target.poseAmbiguity > PhotonvisionConstants.photonUnitAmbiguityCutoff) {
        continue;
      }

      Transform3d transform = target.getBestCameraToTarget();

      translationSums = translationSums.plus(transform.getTranslation());
      cameraToTagRotation = transform.getRotation();
      transformCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // rotations are hard to average (the "average" of two 180 degree rotations is
    // 0, for example). we just take the latest result.
    Transform3d cameraToTag = new Transform3d(translationSums.div(transformCount), cameraToTagRotation);
    Transform3d robotToCamera = robotToTag.plus(cameraToTag.inverse());

    System.out.println("");
    System.out.println("=====================");
    System.out.println("Camera Offset Results");
    System.out.println("=====================");
    System.out.println("Robot To Camera Translation: new Translation3d(" + robotToCamera.getTranslation().getX() + ", "
        + (-robotToCamera.getTranslation().getY()) + ", " + robotToCamera.getTranslation().getZ() + ")");
    System.out.println("Robot To Camera Rotation: new Rotation3d(" + robotToCamera.getRotation().getX() + ", "
        + robotToCamera.getRotation().getY() + ", " + robotToCamera.getRotation().getZ() + ")");
    System.out.println("=====================");
    System.out.println("");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

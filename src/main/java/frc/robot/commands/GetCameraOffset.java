// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Queue;

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
  private List<Rotation3d> cameraToTagRotations = new ArrayList<Rotation3d>();
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
    cameraToTagRotations = new ArrayList<Rotation3d>();
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
      cameraToTagRotations.add(cameraToTagRotation);
      transformCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // rotations are hard to average (the "average" of two 180 degree rotations is
    // 0, for example). we just take the latest result.

    // H! New, untested code to average a bunch of rotations. There's fancy stuff
    // you could do with averaging the rotation matrices and then renormalizing and
    // orthogonalizing them, but we just look at all of them, and interpolate between
    // them in a binary tree fashion. This isn't perfect, but should be pretty good.
    Rotation3d averageRotation = averageRotations(cameraToTagRotations);
    
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

  /** 
   * Calculates an "average" rotation by combining pairs of rotations until only one remains.
   * @param rotations The list of rotations to average. Will be shuffled.
   * @return The "average" rotation. Not a precise notion of average.
   * @author Hale Barber
   */
  private Rotation3d averageRotations(List<Rotation3d> rotations) {
    Collections.shuffle(rotations);
    Queue<Rotation3d> rotationQueue = new ArrayDeque<Rotation3d>(rotations);

    // Repeatedly averages the first two rotations, putting the result on the back
    // of the queue. This ensures the most "raw" (fewest number of applied averagings)
    // Rotations are selected first.
    while (rotationQueue.size() > 1) {
      // Pulls the first two rotations and finds their midpoint. This is then added to the back.
      rotationQueue.add
      (
        rotationQueue.poll().interpolate(rotationQueue.poll(), 0.5)
      );
    }

    return rotationQueue.poll();
  }
}

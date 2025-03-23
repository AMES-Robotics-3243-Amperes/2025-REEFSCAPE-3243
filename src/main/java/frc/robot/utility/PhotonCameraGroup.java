// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** 
 * <h2>WARNING, untested code!</h2>
 * Combines camera data from multiple cameras, applying their offsets manually to allow
 * PNP applied between different cameras.
 */
public class PhotonCameraGroup {
    static public class Measurement {
        public Measurement(EstimatedRobotPose pose, double ambiguity) {
            this.pose = pose;
            this.ambiguity = ambiguity;
        }

        public EstimatedRobotPose pose;
        public double ambiguity;
    }

    /** Includes all data needed to operate a photon camera. */
    static public class PhotonCameraSetup {
        public PhotonCameraSetup(PhotonCamera cam, Transform3d transform) {
            this.cam = cam;
            this.transform = transform;
        }

        public PhotonCamera cam;
        public Transform3d transform;
    }

    public PhotonCameraGroup(PhotonCameraSetup... cameras) {
        this.cameras = cameras;
        estimator = new PhotonPoseEstimator
        (
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), 
            PoseStrategy.CONSTRAINED_SOLVEPNP, 
            new Transform3d()
        );
    }

    private PhotonCameraSetup[] cameras;
    private PhotonPoseEstimator estimator;

    public Optional<Measurement> getMeasurement(Rotation3d headingData, double headingTimestampSeconds) {
        List<Double> ambiguities = new ArrayList<Double>();
        List<PhotonTrackedTarget> adjustedTargets = new ArrayList<>();
        PhotonPipelineMetadata metadata = new PhotonPipelineMetadata();
        boolean hasMetadata = false;
        for (int i = 0; i < cameras.length; i++) {
            final int fI = i;
            var pipelineResults = cameras[i].cam.getAllUnreadResults();

            if (!hasMetadata) {
                metadata = pipelineResults.get(0).metadata;
                hasMetadata = true;
            }

            for (PhotonPipelineResult pipelineResult : pipelineResults) {
                adjustedTargets.addAll(pipelineResult.getTargets().stream().map((t) -> shiftTarget(t, cameras[fI].transform)).toList());
                ambiguities.addAll(pipelineResult.getTargets().stream().map((t) -> t.poseAmbiguity).toList());
            }
        }

        PhotonPipelineResult combinedResult = new PhotonPipelineResult(metadata, adjustedTargets, Optional.empty());
        estimator.addHeadingData(headingTimestampSeconds, headingData);
        Optional<EstimatedRobotPose> result = estimator.update(combinedResult, Optional.empty(),Optional.empty(), Optional.of(new ConstrainedSolvepnpParams(false, 4.0)));
        if (result.isPresent()) {
            double averageAmbiguity = 0.0;
            for (Double ambiguity : ambiguities) {
                averageAmbiguity += ambiguity;
            }
            // divide the mean by the number of targets ^2, because more targets makes the pose significantly better.
            averageAmbiguity /= ambiguities.size();
            averageAmbiguity /= ambiguities.size();
            averageAmbiguity /= ambiguities.size();

            return Optional.of(new Measurement(result.get(), averageAmbiguity));
        }
        return Optional.empty();
    }

    private static PhotonTrackedTarget shiftTarget(PhotonTrackedTarget target, Transform3d transform) {
        Rotation3d targetRot = new Rotation3d(target.skew, target.pitch, target.yaw);
        targetRot = targetRot.rotateBy(transform.getRotation());

        return new PhotonTrackedTarget
        (
            targetRot.getZ(), 
            targetRot.getY(), 
            target.area, 
            targetRot.getX(), 
            target.fiducialId, 
            target.objDetectId, 
            target.objDetectConf, 
            transform.plus(target.bestCameraToTarget), 
            transform.plus(target.altCameraToTarget), 
            target.poseAmbiguity, 
            target.minAreaRectCorners, 
            target.detectedCorners
        );
    }
}

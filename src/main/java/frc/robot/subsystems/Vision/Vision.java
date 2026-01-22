
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;

/**
 * Vision subsystem for handling multiple cameras and robot pose estimation.
 * This subsystem validates poses, computes uncertainties, and notifies consumers.
 */
public class Vision extends SubsystemBase {

  /**
   * Functional interface for receiving new vision estimates.
   * Implement this to respond to valid vision poses.
   */
  @FunctionalInterface
  public interface VisionConsumer {
    void acceptNew(Pose2d visionEstimate, double timestamp, Matrix<N3,N1> visionStdDevs);

    /** Default no-op consumer for convenience */
    VisionConsumer NO_OP = (visionEstimate, timestamp, visionStdDevs) -> {};
  }

  /**
   * Internal representation of a vision camera.
   * Encapsulates the camera interface, its inputs, and calibration constants.
   */
  private static class VisionCamera {
    /** Vision interface implementation (e.g., PhotonVision) */
    public final VisionIO camera;

    /** Auto-logged inputs for this camera */
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    /** Calibration and configuration constants for this camera */
    private CamerasConstants constants;

    /** Constructor for a VisionCamera */
    public VisionCamera(VisionIO camera, String cameraName, CamerasConstants constants) {
      this.camera = camera;
      this.constants = constants;
    }

    /**
     * Factory method for creating a VisionCamera using a PhotonVision implementation.
     *
     * @param constants Camera calibration constants
     * @param lastPoseSupplier Supplier for the last estimated robot pose
     */
    public static VisionCamera fromPhoton(CamerasConstants constants, Supplier<Pose2d> lastPoseSupplier) {
      return new VisionCamera(new VisionIOPhoton(constants, lastPoseSupplier), constants.CAMERA_NAME, constants);
    }

    /**
     * Updates camera inputs and logs them using AdvantageKit.
     */
    public void updateInputs() {
      camera.updateInputs(inputs);
      Logger.processInputs(constants.CAMERA_NAME, inputs);
    }
  }

  /** List of all vision cameras in the system */
  private final List<VisionCamera> cameras;

  /** Consumer that receives valid vision pose estimates */
  private final VisionConsumer poseConsumer;

  /**
   * Creates a new Vision subsystem.
   *
   * @param estimateListener Consumer to handle new vision estimates
   * @param lastPoseSupplier Supplier for the last estimated robot pose
   */
  public Vision(VisionConsumer estimateListener, Supplier<Pose2d> lastPoseSupplier) {
      this.cameras = Arrays.stream(CamerasConstants.values())
          .map(constants -> VisionCamera.fromPhoton(constants,lastPoseSupplier))
          .toList();
      this.poseConsumer = estimateListener;
  }

  /**
   * Utility method to check if a value lies within a given range.
   *
   * @param value Value to check
   * @param min Minimum valid value
   * @param max Maximum valid value
   * @return true if value is within [min, max], false otherwise
   */
  private boolean isWithinRange(double value, double min, double max) {
    return value >= min && value <= max;
  }

  /**
   * Validates whether a Pose3d is within the field boundaries.
   *
   * @param pose Pose to validate
   * @return true if the pose is within valid field bounds
   */
  private boolean isValidLocation(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();

    return isWithinRange(x,0, VisionConstants.FIELD_LAYOUT.getFieldLength()) &&
           isWithinRange(y,0, VisionConstants.FIELD_LAYOUT.getFieldWidth()) &&
           Math.abs(z) <= VisionConstants.MAX_HEIGHT_DEV;
  }

  /**
   * Checks if the pose ambiguity is within acceptable limits.
   *
   * @param ambiguity Pose ambiguity value
   * @param frame Vision frame that contains the number of targets
   * @return true if the ambiguity is valid
   */
  private boolean isValidAmbiguity(double ambiguity, VisionIO.VisionFrame frame) {
    return ambiguity <= (frame.numTargets() > 1 
          ? VisionConstants.MAX_MULTI_AMBIGUTY
          : VisionConstants.MAX_SINGLE_AMBIGUTY);
  }

  /**
   * Computes standard deviations for a pose measurement based on average distance
   * to targets, number of targets, and camera constants.
   *
   * @param avragedistance Average distance to targets
   * @param numTargets Number of targets detected
   * @param camera The camera providing the measurement
   * @return A 3x1 matrix of [xStdDev, yStdDev, thetaStdDev]
   */
  private Matrix<N3,N1> getStdDevs(double avragedistance,int numTargets, VisionCamera camera) {
    if(numTargets <= 0) {
      // Return NaN if no targets detected to indicate unknown uncertainty
      return VecBuilder.fill(camera.constants.MAX_XY_STD_DEV, camera.constants.MAX_XY_STD_DEV, camera.constants.MAX_THETA_STD_DEV);
    }

    double stdDevFactor = Math.pow(avragedistance, 2)/numTargets;

    double stdDevLinear = Math.max(
      stdDevFactor * camera.constants.XY_STD_DEV_FACTOR,
      camera.constants.MIN_XY_STD_DEV
    );

    double stdDevAngular = Math.max(
      stdDevFactor * camera.constants.THETA_STD_DEV_FACTOR,
      camera.constants.MIN_THETA_STD_DEV
    );

    return VecBuilder.fill(stdDevLinear, stdDevLinear, stdDevAngular);
  }

  /**
   * Periodically called by the scheduler.
   * Updates camera inputs, validates poses, computes uncertainties,
   * notifies the consumer, and logs data.
   */
  @Override
  public void periodic() {
    List<Pose3d> validPoses = new ArrayList<>(); 
    List<Pose3d> invalidPoses = new ArrayList<>();
    List<Translation3d> targetLocations = new ArrayList<>();

    for(VisionCamera camera : cameras) {
      // Update inputs from camera and log connection status
      camera.updateInputs();
      SmartDashboard.putBoolean(camera.constants.CAMERA_NAME + "/connected", camera.inputs.isConnected);
      if(!camera.inputs.isConnected) continue; // Skip disconnected cameras

      // Collect positions of all detected fiducials
      for(int id: camera.inputs.targetIDs) {
        VisionConstants.FIELD_LAYOUT.getTagPose(id)
        .ifPresent(tagPose -> targetLocations.add(tagPose.getTranslation()));
      }

      // Process each vision frame
      for(VisionIO.VisionFrame frame : camera.inputs.visionFrames) {
        if(!frame.hasTarget()) continue; // Skip empty frames

        Pose3d pose = frame.targetPose();

        // Validate location and ambiguity
        if(!isValidLocation(pose) || !isValidAmbiguity(frame.targetPoseAmbiguity(), frame)) {
          invalidPoses.add(pose);
          continue;
        }

        // Compute standard deviations for the measurement
        var stdDevs = getStdDevs(frame.avrageDistanceToTargetsMeters(), frame.numTargets(), camera);

        // Notify the consumer of a valid pose estimate
        poseConsumer.acceptNew(pose.toPose2d(), frame.timeStampSeconds(), stdDevs);

        // Track valid poses for logging
        validPoses.add(pose);
      }
    }

    // Log target locations and valid/rejected poses for debugging
    Logger.recordOutput("Vision/Target Locations", targetLocations.toArray(new Translation3d[0]));
    Logger.recordOutput("Vision/Valid Poses", validPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Rejected Poses", invalidPoses.toArray(new Pose3d[0]));
  }
}

package frc.robot.subsystems.Vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;

/**
 * A simulated version of the VisionIOPhoton class
 * Uses PhotonVision's simulation tools to replicate camera behavior
 * for use in the FRC simulator.
 */
public class VisionIOPhotonSim extends VisionIOPhoton {

    // Simulation of the whole vision system (AprilTags, cameras, etc.)
    VisionSystemSim visionSim;

    // Simulated camera
    PhotonCameraSim cameraSim;

    // Supplies the latest robot pose (used to update sim)
    private final Supplier<Pose2d> lastPoseSupplier;

    // Actual PhotonVision camera reference (even though we're simulating)
    PhotonCamera camera;


    /**
     * Constructor for the simulated Vision I/O
     * @param camConst - constants/config for this camera
     * @param lastPoseSupplier - function to provide the robot's latest pose
     */
    public VisionIOPhotonSim(CamerasConstants camConst, Supplier<Pose2d> lastPoseSupplier) {
        // Call the parent constructor
        super(camConst, lastPoseSupplier);
        this.lastPoseSupplier = lastPoseSupplier;

        // Create a new VisionSystemSim if it doesn't exist yet
        if(visionSim == null){
            visionSim = new VisionSystemSim("main");
            // Add AprilTags to the simulation using the field layout
            visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        }

        // Define simulated camera properties (resolution, FOV, latency, etc.)
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(91.1)); // resolution + diagonal FOV
        cameraProp.setAvgLatencyMs(20); // simulate network/processing delay (tunable)
        
        // Create a PhotonCamera instance with the configured camera name
        camera = new PhotonCamera(camConst.CAMERA_NAME);
        
        // Create a simulated PhotonCamera based on our properties
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addCamera(cameraSim, CamerasConstants.FRONT_CAMERA.ROBOT_TO_CAMERA);
    }


    /**
     * Updates the simulated vision system
     * @param inputs - the data structure where vision results will be stored
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        // Update the vision simulation with the robot's latest pose
        visionSim.update(lastPoseSupplier.get());

        // Call the parent class to handle the rest of the data update
        super.updateInputs(inputs);
    }

    /**
     * @return the camera's name (from its configuration)
     */
    public String getName(){
        return camera.getName();
    }
}
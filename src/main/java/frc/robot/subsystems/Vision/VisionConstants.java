
package frc.robot.subsystems.Vision;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class VisionConstants {
      /**
     * Max height deviation for vision targets (in meters)
     */
    public static final double MAX_HEIGHT_DEV = 0.1;
    /**
     * Max ambiguity for multi-tag targets (0,one hunderd precent sure to 1, random.next)
     */
    public static final double MAX_MULTI_AMBIGUTY = 0.3;
    /**
     * Max ambiguity for single-tag targets (0 to 1)
     */
    public static final double MAX_SINGLE_AMBIGUTY = 0.1;

    public record StdDevsFactors(double xyStdFactor,double thetaStdFactor,double minXyStd,double minThetaStd) {
    }


    // enum with all the camera constants
    enum CamerasConstants {
        // Define the camera constants for the front camera
        FRONT_CAMERA(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            "dolev",
            new Transform3d(
                new Translation3d(-0.145, -0.345, 0.445),
                new Rotation3d(0, Units.degreesToRadians(-35),Units.degreesToRadians(180))
    
            ),
            new StdDevsFactors(0.1,0.3,0.1,0.3),
            new StdDevsFactors(0.15,0.35,0.1,0.3)
            
        );
        
       
        /**
         * The main strategy for the camera
         */
        public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        
       /**
        * The alternate strategy for the camera
        */
        public final PhotonPoseEstimator.PoseStrategy ALTERNATE_STRATEGY;

        /**
         * The name of the camera
         */
        public final String CAMERA_NAME;
        
                /**
                 * The transform from the robot to the camera
                 */ 
                public final Transform3d ROBOT_TO_CAMERA;
        
                public final StdDevsFactors[] stdDevsFactors;
            
        
                // Constructor for the camera constants
                CamerasConstants(
                PhotonPoseEstimator.PoseStrategy alternateStrategy, 
                String cameraName, 
                Transform3d robotToCamera,
                StdDevsFactors... factors) {
                    
        
                    this.ALTERNATE_STRATEGY = alternateStrategy;
        
                    this.CAMERA_NAME = cameraName;

            this.ROBOT_TO_CAMERA = robotToCamera;

            stdDevsFactors = factors;

            if(factors.length < 1){
                throw new IllegalArgumentException("must provide at least 1 std Devs factor");
            }
        }


        
    }
    /**
     * The field layout for the 2026 FRC game "Rebuilt"
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
}

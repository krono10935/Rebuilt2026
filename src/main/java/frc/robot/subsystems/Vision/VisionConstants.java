
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


    // enum with all the camera constants
    enum CamerasConstants {
        // Define the camera constants for the front camera
        FRONT_CAMERA(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            "dolev",
            new Transform3d(
                new Translation3d(0.25, 0.065, 0.08),
                new Rotation3d(0, Units.degreesToRadians(-55),0)
    
            ),
            0.1, // XY standard deviation factor
            0.1, // Theta standard deviation factor
             0.05,// Minimum XY standard deviation
              Math.toRadians(5)// Minimum Theta standard deviation
            
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
        
                /**
                 * The standard deviation factor for XY
                 */
                public final double XY_STD_DEV_FACTOR;
        
                /**
                 * The standard deviation factor for Theta
                 */
                public final double THETA_STD_DEV_FACTOR;
                /**
                 * The minimum standard deviation for XY (in meters)
                 */
                public final double MIN_XY_STD_DEV; 
                /**
                 * The minimum standard deviation for Theta (in radians)
                 */
                public final double MIN_THETA_STD_DEV;
                
                public final double MAX_XY_STD_DEV = 200; // Maximum XY standard deviation (in meters)
                public final double MAX_THETA_STD_DEV = Math.toRadians(180); // Maximum Theta standard deviation (in radians)
        
                // Constructor for the camera constants
                CamerasConstants(
                PhotonPoseEstimator.PoseStrategy alternateStrategy, 
                String cameraName, 
                Transform3d robotToCamera,
                double xyStdFactor,
                double thetaStdFactor,
                double minXYStd,
                double minThetaStd) {
                    
        
                    this.ALTERNATE_STRATEGY = alternateStrategy;
        
                    this.CAMERA_NAME = cameraName;

            this.ROBOT_TO_CAMERA = robotToCamera;

            //TODO: determaine unit for standard deviation factor
            this.XY_STD_DEV_FACTOR = xyStdFactor;

            this.THETA_STD_DEV_FACTOR = thetaStdFactor;

            this.MIN_XY_STD_DEV = minXYStd;

            this.MIN_THETA_STD_DEV = minThetaStd;
        }


        
    }
    /**
     * The field layout for the 2025 FRC game "Reefscape"
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
}

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.ShootCalculatorWithMovement.ShootCalculatorWithMovementParams;
import frc.utils.AllianceFlipUtil;

public class ShotCalculator {
    private static ShotCalculator instance;

    public enum ValidityState{
        VALID("Valid", AlertType.kInfo),
        OUT_OF_RANGE("Out of shooting range", AlertType.kWarning),
        TOO_MUCH_OMEGA_SPEED("Stop spinning", AlertType.kWarning),
        HUB_INACTIVE("Womp womp the hub is inactive", AlertType.kWarning),
        SHOULD_NOT_BE_MOVING("Stop moving", AlertType.kWarning);

        private final Alert alert;

        ValidityState(String message, AlertType alertType){
            alert = new Alert(message, alertType);
        }

        public void toggleAlert(boolean activate){
            alert.set(activate);
        }
    }


    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
        ValidityState validityState,
        Rotation2d robotAngle,
        Rotation2d hoodAngle,
        double flywheelSpeed) {}
        
    private ShootingParameters latestParameters = null;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;

    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();

    static {
        minDistance = 1.34; // TODO: find the real value
        maxDistance = 5.60; // TODO: find the real value
        phaseDelay = 0.03; // TODO: find the real value

        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));


        shotFlywheelSpeedMap.put(1.34, 210.0/60);
        shotFlywheelSpeedMap.put(1.78, 220.0/60);
        shotFlywheelSpeedMap.put(2.17, 220.0/60);
        shotFlywheelSpeedMap.put(2.81, 230.0/60);
        shotFlywheelSpeedMap.put(3.82, 250.0/60);
        shotFlywheelSpeedMap.put(4.09, 255.0/60);
        shotFlywheelSpeedMap.put(4.40, 260.0/60);
        shotFlywheelSpeedMap.put(4.77, 265.0/60);
        shotFlywheelSpeedMap.put(5.57, 275.0/60);
        shotFlywheelSpeedMap.put(5.60, 290.0/60);

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);

        // shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); // Find real measurements

        // shotFlywheelSpeedMap.put(1.34,210.0); // Find real measurement

        // timeOfFlightMap.put(5.68, 1.16); // Find real measurement
    }

    public void warmUpShotCalculator(){
        Pose2d warmUpPose = new Pose2d();
        ChassisSpeeds robotRelativeChassisSpeeds = new ChassisSpeeds();

        getParameters(warmUpPose, robotRelativeChassisSpeeds);
        clearShootingParameters();

        warmUpPose = new Pose2d(1,1,Rotation2d.fromDegrees(90));
        robotRelativeChassisSpeeds = new ChassisSpeeds(1,0,90);

        getParameters(warmUpPose, robotRelativeChassisSpeeds);
        clearShootingParameters();

        warmUpPose = new Pose2d(-10,32,Rotation2d.fromDegrees(67));
        robotRelativeChassisSpeeds = new ChassisSpeeds(10,9,35);

        getParameters(warmUpPose, robotRelativeChassisSpeeds);
        clearShootingParameters();
    }

    /**
     * 
     * @param estimatedPoseSupplier Estimated robot pose
     * @param robotRelativeVelocitySupplier Robot-relative velocity
     * @return ShootingParameters based on these parameters
     */
    public ShootingParameters getParameters(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
        if (latestParameters != null){
            return latestParameters;
        }
        
        Pose2d shooterPosition = shooterPoseAtShooting(estimatedPose, robotRelativeVelocity);
        
        Translation2d hub = 
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
     
        Translation2d shooterFieldRelativeSpeeds = 
            getShooterFieldRelativeSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation()),
            shooterPosition);

        if (ShooterConstants.SHOOT_WITH_MOVEMENT){

            return calculateShootingParametersWithMovement(shooterPosition, shooterFieldRelativeSpeeds, hub, robotRelativeVelocity);

        } else {

            return calculateShootingParametersWithoutMovement(shooterPosition, shooterFieldRelativeSpeeds, hub, robotRelativeVelocity);
        }
    }

    /**
     * @param robotRelativeVelocity The robot relative velocity
     * @param shooterFieldRelativeSpeeds field relative X and Y speed of the robot (and therefore the shooter)
     * @param shootWithMovmentParams The shoot with movement params
     * @return the validity state of the calculation
     */
    @SuppressWarnings("unused")
    private ValidityState findValidityState(ChassisSpeeds robotRelativeVelocity, Translation2d shooterFieldRelativeSpeeds,
     double lookaheadShooterToTargetDistance){

        ValidityState state = ValidityState.VALID;

        // if (!Constants.HubTiming.isActive(DriverStation.getMatchTime() + timeOfFlightMap.get(lookaheadShooterToTargetDistance))){
        //     state = ValidityState.HUB_INACTIVE;
        // }

        if (Math.abs(robotRelativeVelocity.omegaRadiansPerSecond) > ShooterConstants.ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES){
                state = ValidityState.TOO_MUCH_OMEGA_SPEED;
        }

        if (!ShooterConstants.SHOOT_WITH_MOVEMENT && Math.abs(shooterFieldRelativeSpeeds.getNorm()) > ShooterConstants.ZERO_LINEAR_SPEED_TOLERANCE_MPS){
            state = ValidityState.SHOULD_NOT_BE_MOVING;
        }

        if (lookaheadShooterToTargetDistance <= minDistance ||
            lookaheadShooterToTargetDistance >= maxDistance){
                state = ValidityState.OUT_OF_RANGE;
        }

        return state;
    }

    /**
     * 
     * @param estimatedPose robot estimated pose
     * @param robotRelativeVelocity robot relative velocity of the robot
     * @return The pose the robot will actually have at shooting
     */
    private Pose2d shooterPoseAtShooting(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
        estimatedPose =  estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
            ));

        Transform2d transformRobotToShooter = 
        new Transform2d(
            ShooterConstants.ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(),
            Rotation2d.fromRadians(
                ShooterConstants.ROBOT_TO_SHOOTER.getRotation().getZ()
            )
        );

        // Calculate distance from the shooter to the target
        Pose2d shooterPosition = estimatedPose.transformBy(transformRobotToShooter); 

        return shooterPosition;
    }

    /**
     * 
     * @param robotVelocityFieldRelative the robot velocity relative to the field
     * @param estimatedRobotPose the current estimated robot pose
     * @return Translation2d object encompassing the field relative X and Y speeds
     */
    private Translation2d getShooterFieldRelativeSpeeds(ChassisSpeeds robotVelocityFieldRelative, Pose2d estimatedShooterPose){
        
        Translation2d linearSpeed = new Translation2d(
            robotVelocityFieldRelative.vxMetersPerSecond,
            robotVelocityFieldRelative.vyMetersPerSecond
        );

        double tangentRobotAngularSpeed = robotVelocityFieldRelative.omegaRadiansPerSecond * 
            ShooterConstants.ROBOT_TO_SHOOTER.getTranslation().getNorm();
        
        Rotation2d tangentRobotAngleFieldRelative = estimatedShooterPose.getRotation().rotateBy(Rotation2d.kCCW_90deg);
        
        Translation2d angularToLinearFieldRelativeSpeed = new Translation2d(
            tangentRobotAngularSpeed, tangentRobotAngleFieldRelative
        );

        Logger.recordOutput("ShotCalculator/AngularToFieldRelative", angularToLinearFieldRelativeSpeed);
        Logger.recordOutput("ShotCalculator/LinearSpeeds", linearSpeed);

        return linearSpeed.plus(angularToLinearFieldRelativeSpeed);

    }


    /**
     * 
     * @param shooterPosition Position of the shooter
     * @param shooterFieldRelativeSpeeds Speed of the shooter on the field
     * @param hub Translation 2d of where the hub is located
     * @param robotRelativeVelocity Robot relative velocity of the robot, which is also shooter's velocity
     * @return Shooting params calculated with movement
     */
    private ShootingParameters calculateShootingParametersWithMovement(Pose2d shooterPosition, Translation2d shooterFieldRelativeSpeeds,
     Translation2d hub, ChassisSpeeds robotRelativeVelocity){
        
        ShootCalculatorWithMovementParams shootWithMovementParams = 
                ShootCalculatorWithMovement.regressFuturePositionParams(shooterPosition, timeOfFlightMap,
                shooterFieldRelativeSpeeds, hub);
        
        Logger.recordOutput("ShotCalculator/shootWithMovementParams", shootWithMovementParams);
            
        Pose2d lookaheadPose = shootWithMovementParams.lookaheadPose();
        double lookaheadShooterToTargetDistance = shootWithMovementParams.lookaheadShooterToTargetDistance();

            // Find the robot angle to shoot at
        Rotation2d robotAngle = hub.minus(lookaheadPose.getTranslation()).getAngle();

        // Calculate the optimal hood angle
        var hoodAngle = shotHoodAngleMap.get(lookaheadShooterToTargetDistance);

        ValidityState state = findValidityState(robotRelativeVelocity, shooterFieldRelativeSpeeds, lookaheadShooterToTargetDistance);

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);


        Logger.recordOutput("ShotCalculator/output", latestParameters);


        return latestParameters;

    }

    /**
     * 
     * @param shooterPosition Position of the shooter
     * @param shooterFieldRelativeSpeeds Speed of the shooter on the field
     * @param hub Translation 2d of where the hub is located
     * @param robotRelativeVelocity Robot relative velocity of the robot, which is also shooter's velocity
     * @return Shooting params calculated without movement
     */
    public ShootingParameters calculateShootingParametersWithoutMovement(Pose2d shooterPosition, Translation2d shooterFieldRelativeSpeeds,
     Translation2d hub, ChassisSpeeds robotRelativeVelocity){
        
        double distanceFromHub = shooterPosition.getTranslation().getDistance(hub);

        Rotation2d robotAngle = hub.minus(shooterPosition.getTranslation()).getAngle();

        // Calculate the optimal hood angle
        var hoodAngle = shotHoodAngleMap.get(distanceFromHub);

        ValidityState state = findValidityState(robotRelativeVelocity, shooterFieldRelativeSpeeds, distanceFromHub);

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(distanceFromHub));


        Logger.recordOutput("ShotCalculator/output", latestParameters);

        return latestParameters;
    }

    /**
     * Clear shooting parameters to allow recalculation
     */
    public void clearShootingParameters(){
        latestParameters = null;
    }

}

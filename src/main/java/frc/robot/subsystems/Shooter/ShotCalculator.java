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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.ShootCalculatorWithMovement.ShootCalculatorWithMovementParams;
import frc.utils.AllianceFlipUtil;

public class ShotCalculator {
    private static ShotCalculator instance;

    public enum ValidityState{
        VALID,
        OUT_OF_RANGE,
        TOO_MUCH_OMEGA_SPEED,
        HUB_INACTIVE
    }
    private Rotation2d lastRobotAngle;
    private double lastHoodAngle;
    private Rotation2d robotAngle;
    private double hoodAngle = Double.NaN;


    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
        ValidityState validityState,
        Rotation2d robotAngle,
        double hoodAngle,
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

        shotFlywheelSpeedMap.put(1.34, 210.0);
        shotFlywheelSpeedMap.put(1.78, 220.0);
        shotFlywheelSpeedMap.put(2.17, 220.0);
        shotFlywheelSpeedMap.put(2.81, 230.0);
        shotFlywheelSpeedMap.put(3.82, 250.0);
        shotFlywheelSpeedMap.put(4.09, 255.0);
        shotFlywheelSpeedMap.put(4.40, 260.0);
        shotFlywheelSpeedMap.put(4.77, 265.0);
        shotFlywheelSpeedMap.put(5.57, 275.0);
        shotFlywheelSpeedMap.put(5.60, 290.0);

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);

        // shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); // Find real measurements

        // shotFlywheelSpeedMap.put(1.34,210.0); // Find real measurement

        // timeOfFlightMap.put(5.68, 1.16); // Find real measurement
    }

    /**
     * 
     * @param estimatedPoseSupplier Estimated robot pose
     * @param robotRelativeVelocitySupplier Robot-relative velocity
     * @param robotFieldVelocitySupplier Field-relative velocity
     * @return ShootingParameters based on these parameters
     */
    public ShootingParameters getParameters(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity, ChassisSpeeds robotVelocityFieldRelative){
        if (latestParameters != null){
            return latestParameters;
        }
        
        estimatedPose = poseAtShooting(estimatedPose, robotRelativeVelocity);
        
        Translation2d hub = 
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        Transform2d transformRobotToShooter = 
            new Transform2d(
                ShooterConstants.ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(),
                Rotation2d.fromRadians(
                    ShooterConstants.ROBOT_TO_SHOOTER.getRotation().getZ()
                )
            );

        // Calculate distance from the shooter to the target
        Pose2d shooterPosition = estimatedPose.transformBy(transformRobotToShooter);       

        Translation2d shooterFieldRelativeSpeeds = getShooterFieldRelativeSpeeds(robotRelativeVelocity,
         transformRobotToShooter.getTranslation(),
         estimatedPose);

        ShootCalculatorWithMovementParams shootWithMovmentParams = 
        ShootCalculatorWithMovement.regressFuturePositionParams(shooterPosition, timeOfFlightMap,
         shooterFieldRelativeSpeeds, hub);

        // Find the robot angle to shoot at
        robotAngle = hub.minus(shootWithMovmentParams.lookaheadPose().getTranslation()).getAngle();

        // Calculate the optimal hood angle
        hoodAngle = shotHoodAngleMap.get(shootWithMovmentParams.lookaheadShooterToTargetDistance()).getRadians();
        if (lastRobotAngle == null) lastRobotAngle = robotAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

        lastRobotAngle = robotAngle;
        lastHoodAngle = hoodAngle;

        ValidityState state = findValidityState(robotRelativeVelocity, shootWithMovmentParams);

        Constants.HubTiming.setStartingTeam(DriverStation.getGameSpecificMessage(), DriverStation.getAlliance().get());

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(shootWithMovmentParams.lookaheadShooterToTargetDistance()));

        Logger.recordOutput("ShotCalculator/LookaheadPose", shootWithMovmentParams.lookaheadPose());
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", shootWithMovmentParams.lookaheadShooterToTargetDistance());

        return latestParameters;
    }

    /**
     * @param robotRelativeVelocity The robot relative velocity
     * @param shootWithMovmentParams The shoot with movement params
     * @return the validity state of the calculation
     */
    private ValidityState findValidityState(ChassisSpeeds robotRelativeVelocity,
     ShootCalculatorWithMovementParams shootWithMovmentParams){

        ValidityState state = ValidityState.VALID;

        if (!Constants.HubTiming.isActive(DriverStation.getMatchTime())){
            state = ValidityState.HUB_INACTIVE;
        }

        if (Math.abs(robotRelativeVelocity.omegaRadiansPerSecond) > ShooterConstants.ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES){
                state = ValidityState.TOO_MUCH_OMEGA_SPEED;
        }

        if (shootWithMovmentParams.lookaheadShooterToTargetDistance() >= minDistance &&
            shootWithMovmentParams.lookaheadShooterToTargetDistance() <= maxDistance){
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
    private Pose2d poseAtShooting(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
         return estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
            ));
    }

    /**
     * 
     * @param robotVelocityFieldRelative the robot velocity relative to the field
     * @param robotToShooter translation from the robot to the shooter
     * @param estimatedRobotPose the current estimated robot pose
     * @return Translation2d object encompassing the field relative X and Y speeds
     */
    private Translation2d getShooterFieldRelativeSpeeds(ChassisSpeeds robotVelocityFieldRelative,
     Translation2d robotToShooter, Pose2d estimatedRobotPose){
        Translation2d linearSpeed = new Translation2d(
            robotVelocityFieldRelative.vxMetersPerSecond,
            robotVelocityFieldRelative.vyMetersPerSecond
        );

        double tangentRobotAngularSpeed = robotVelocityFieldRelative.omegaRadiansPerSecond * robotToShooter.getNorm();
        Rotation2d tangentRobotAngleFieldRelative = estimatedRobotPose.getRotation().plus(Rotation2d.kCW_90deg);
        
        Translation2d angularToLinearFieldRelativeSpeed = new Translation2d(
            tangentRobotAngularSpeed, tangentRobotAngleFieldRelative
        );

        return linearSpeed.plus(angularToLinearFieldRelativeSpeed);

    }

    /**
     * Clear shooting parameters to allow recalculation
     */
    public void clearShootingParameters(){
        latestParameters = null;
    }

}

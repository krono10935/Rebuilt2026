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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private static final double minDistance;
    private static final double maxDistance;
    private static final double phaseDelay;

    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();

    /**
     *
     * @param distance
     * @param angleDegrees
     * @param shotSpeed
     * @param timeOfFlight
     */
    private static void putToMaps(double distance, double angleDegrees, double shotSpeed, double timeOfFlight){
        distance = distance - ShooterConstants.ROBOT_TO_SHOOTER.getX();
        shotHoodAngleMap.put(distance, Rotation2d.fromDegrees(angleDegrees));
        shotFlywheelSpeedMap.put(distance,shotSpeed);
        timeOfFlightMap.put(distance,timeOfFlight);
    }

    private static void staticPutToMapsNoTransform(double distance, double angleDegrees, double shotSpeed, double timeOfFlight){
        shotHoodAngleMap.put(distance, Rotation2d.fromDegrees(angleDegrees));
        shotFlywheelSpeedMap.put(distance,shotSpeed);
        timeOfFlightMap.put(distance,timeOfFlight);
    }
    static {
        minDistance = 1; // TODO: find the real value
        maxDistance = 5.60; // TODO: find the real value
        phaseDelay = 0.03; // TODO: find the real value

        putToMaps(1.065, 3, 15.0, 0.9);
        putToMaps(1.196, 4, 15.5, 1.0 );
        putToMaps(1.307, 5,15.65, 1.12 );
        putToMaps(1.401, 5,15.5, 1.06 );
        putToMaps(1.510, 6,15.5, 1.09 );
        putToMaps(1.610, 7, 15.5 ,1 );
        putToMaps(1.707, 8, 15.5, 1.03 );
        putToMaps(1.813,9, 15.5, 1);
        putToMaps(1.910, 10 , 16.2 , 1.08);
        putToMaps(2.011, 11, 16.3 , 1.06);
        putToMaps(2.101, 12, 16.4,0.94);
        putToMaps(2.194, 13, 16.4,0.95 );
        putToMaps(2.310 , 14, 16.4, 0.92);
        putToMaps(2.4, 15, 16.4, 0.95);
        putToMaps(2.5, 16, 16.4, 0.94);
        putToMaps(2.6, 17, 17.1,1 );
        putToMaps(2.7, 18, 17.2, 0.9);
        putToMaps(2.8, 19, 17.5, 1.02 );
        putToMaps(2.91, 18, 18, 0.95);
        putToMaps(3,19, 18, 1.02 );
        putToMaps(3.1, 21, 18, 1.01);
        putToMaps(3.2, 22, 18.1, 1.07);
        putToMaps(3.3, 23, 18.1, 1);
        putToMaps(3.4, 25, 18.1, 0.9);
        putToMaps(3.5, 26, 18.1,0.97 );
        putToMaps(3.6, 27,18.1, 1.04);
        putToMaps(3.7 , 28, 18.1, 1.07);
        putToMaps(4.0 , 28, 18.2, 1.2);
        putToMaps(4.22 , 28, 18.55, 1.2);
        putToMaps(4.4 , 29, 23, 1.27);
        putToMaps(4.6 , 29, 25, 1.45);
        putToMaps(4.8 , 29, 27, 1.5);

    }


    public Command warmUpShotCalculator(){
        return new InstantCommand(this::warmUpGeneratePoses);
    }

    /**
     * Generate the chassis speeds and then use it and the pose to calculate the shooting params
     * @param warmUpPose the pose to use to calculate the shooting params
     */
    private void warmUpWithPose(Pose2d warmUpPose){
        for (int speedOmegaRadians = 0; speedOmegaRadians < 360; speedOmegaRadians += 120){
            for (int speedsXYSpeed = 1; speedsXYSpeed < 4; speedsXYSpeed += 1){
                clearShootingParameters();
                getParameters(warmUpPose, new ChassisSpeeds(speedsXYSpeed, speedsXYSpeed, speedOmegaRadians));
            }
        }
    }


    /**
     * Generate the poses from the warm up and for each pose, calculate the shooting params
     */
    private void warmUpGeneratePoses(){
        for (int poseVectorAngleDegrees = 0; poseVectorAngleDegrees < 360; poseVectorAngleDegrees += 120){
            for (int poseVectorSize = 1; poseVectorSize < 4; poseVectorSize += 1){
                for (int poseFieldRelativeAngleDegrees = 0; poseFieldRelativeAngleDegrees < 360; poseFieldRelativeAngleDegrees += 120){
                    warmUpWithPose(
                        new Pose2d(
                            new Translation2d(poseVectorSize, Rotation2d.fromDegrees(poseVectorAngleDegrees))
                        , Rotation2d.fromDegrees(poseFieldRelativeAngleDegrees))
                    );
                }
            }
        }
    }

    public ShootingParameters getStaticParameters(Pose2d estimatedPose){
        return getParameters(estimatedPose, new ChassisSpeeds());
    }
    /**
     * 
     * @param estimatedPose Estimated robot pose
     * @param robotRelativeVelocity Robot-relative velocity
     * @return ShootingParameters based on these parameters
     */
    public ShootingParameters getParameters(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
        if (latestParameters != null){
            return latestParameters;
        }
        
        Pose2d shooterPosition = shooterPoseAtShooting(estimatedPose, robotRelativeVelocity);

        Logger.recordOutput("ShotCalculator/Shooter position", shooterPosition);
        
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
     * @param lookaheadShooterToTargetDistance The shoot with movement params
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
     * @param estimatedShooterPose the current estimated robot pose
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

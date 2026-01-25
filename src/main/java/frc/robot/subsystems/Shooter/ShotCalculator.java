package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.utils.AllianceFlipUtil;

public class ShotCalculator {
    private static ShotCalculator instance;

    private final LinearFilter robotAngleFilter = 
        LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));

        private final LinearFilter hoodAngleFilter = 
            LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));

        private Rotation2d lastRobotAngle;
        private double lastHoodAngle;
        private Rotation2d robotAngle;
        private double hoodAngle = Double.NaN;
        private double robotVelocity;
        private double hoodVelocity;

        public static ShotCalculator getInstance() {
            if (instance == null) instance = new ShotCalculator();
            return instance;
        }

        public record ShootingParameters(
            boolean isValid,
            Rotation2d robotAngle,
            double turretVelocity,
            double hoodAngle,
            double hoodVelocity,
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

        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); // Find real measurements

        shotFlywheelSpeedMap.put(1.34,210.0); // Find real measurement

        timeOfFlightMap.put(5.68, 1.16); // Find real measurement
    }

    /**
     * 
     * @param estimatedPoseSupplier Estimated robot pose
     * @param robotRelativeVelocitySupplier Robot-relative velocity
     * @param robotFieldVelocitySupplier Field-relative velocity
     * @return ShootingParameters based on these parameters
     */
    public ShootingParameters getParameters(Supplier<Pose2d> estimatedPoseSupplier, Supplier<ChassisSpeeds> robotRelativeVelocitySupplier, Supplier<ChassisSpeeds> robotFieldVelocitySupplier){
        if (latestParameters != null){
            return latestParameters;
        }

        Pose2d estimatedPose = estimatedPoseSupplier.get();
        ChassisSpeeds robotRelativeVelocity = robotRelativeVelocitySupplier.get();

        // Estimate where the robot will be by the time the calculation finished
        estimatedPose = 
            estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
            ));

        Translation2d target = 
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        Transform2d transform = 
            new Transform2d(
                ShooterConstants.ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(),
                Rotation2d.fromRadians(
                    ShooterConstants.ROBOT_TO_SHOOTER.getRotation().getZ()
                )
            );

        // Calculate distance from the shooter to the target
        Pose2d shooterPosition = estimatedPose.transformBy(transform);
        double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

        // Find the current robot velocity on the different axis
        ChassisSpeeds robotVelocityCurrent = robotFieldVelocitySupplier.get();
        double robotAngleCurrent = estimatedPose.getRotation().getRadians();
        
        double shooterVelocityX = robotVelocityCurrent.vxMetersPerSecond + 
            robotVelocityCurrent.omegaRadiansPerSecond * (ShooterConstants.ROBOT_TO_SHOOTER.getY() * Math.cos(robotAngleCurrent) 
            - ShooterConstants.ROBOT_TO_SHOOTER.getY() * Math.sin(robotAngleCurrent));
        
        double shooterVelocityY = robotVelocityCurrent.vxMetersPerSecond + 
            robotVelocityCurrent.omegaRadiansPerSecond * (ShooterConstants.ROBOT_TO_SHOOTER.getX() * Math.cos(robotAngleCurrent) 
            - ShooterConstants.ROBOT_TO_SHOOTER.getY() * Math.sin(robotAngleCurrent));

        // Regress to the optimal shooting calculation for distance to shoot
        double timeOfFlight;
        Pose2d lookaheadPose = shooterPosition;
        double lookaheadShooterToTargetDistance = shooterToTargetDistance;
        for (int i = 0; i < 20; i++){
            timeOfFlight = timeOfFlightMap.get(lookaheadShooterToTargetDistance);
            double offsetX = shooterVelocityX * timeOfFlight;
            double offsetY = shooterVelocityY * timeOfFlight;

            lookaheadPose = 
                 new Pose2d(
                    shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    shooterPosition.getRotation());

            lookaheadShooterToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Find the robot angle to shoot at
        robotAngle = target.minus(lookaheadPose.getTranslation()).getAngle();

        // Calculate the optimal hood angle
        hoodAngle = shotHoodAngleMap.get(lookaheadShooterToTargetDistance).getRadians();
        if (lastRobotAngle == null) lastRobotAngle = robotAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

        // Find the optimal robot Velocity to shoot at
        robotVelocity = 
            robotAngleFilter.calculate(
                robotAngle.minus(lastRobotAngle).getRadians() / Constants.LOOP_PERIOD_SECONDS);

        // Find the optimal hood velocity to shoot with
        hoodVelocity = 
            hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_PERIOD_SECONDS);

        lastRobotAngle = robotAngle;
        lastHoodAngle = hoodAngle;

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(lookaheadShooterToTargetDistance >= minDistance &&
            lookaheadShooterToTargetDistance <= maxDistance,
            robotAngle,
            robotVelocity,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadShooterToTargetDistance);

        return latestParameters;
    }

    // Reset shooting parameters, to allow recalculation
    public void clearShootingParameters(){
        latestParameters = null;
    }

}

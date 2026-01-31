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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.utils.AllianceFlipUtil;

public class ShotCalculator {
    private static ShotCalculator instance;

    public enum ValidityState{
        VALID,
        OUT_OF_RANGE,
        TOO_MUCH_OMEGA_SPEED,
        HUB_INACTIVE
    }

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
            ValidityState validityState,
            Rotation2d robotAngle,
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

        //TODO: see if this calcs can be removed eniterly
        double shooterVelocityX = robotVelocityFieldRelative.vxMetersPerSecond;
        
        double shooterVelocityY = robotVelocityFieldRelative.vyMetersPerSecond;

        // Regress to the optimal shooting calculation for distance to shoot
        // We assume that this is to calculate the v0(m/s) that the robot gives the ball in fieldRelative terms
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

        ValidityState state = ValidityState.VALID;

        if (!Constants.HubTiming.isActive(DriverStation.getMatchTime())){
            state = ValidityState.HUB_INACTIVE;
        }

        if (Math.abs(robotRelativeVelocity.omegaRadiansPerSecond) > ShooterConstants.ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES){
                state = ValidityState.TOO_MUCH_OMEGA_SPEED;
        }

        if (lookaheadShooterToTargetDistance >= minDistance &&
            lookaheadShooterToTargetDistance <= maxDistance){
                state = ValidityState.OUT_OF_RANGE;
        }

        Constants.HubTiming.setStartingTeam(DriverStation.getGameSpecificMessage());

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

        return latestParameters;
    }

    // Reset shooting parameters, to allow recalculation
    public void clearShootingParameters(){
        latestParameters = null;
    }

}

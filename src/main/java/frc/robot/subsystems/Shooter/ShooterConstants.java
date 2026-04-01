package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterConstants {

    public static final Transform3d ROBOT_TO_SHOOTER = 
        new Transform3d(0.2, 0.0, 0.4, Rotation3d.kZero); // Find real translation
    
    public static final double FLYWHEEL_CICUMFRENCE = 0.11 * Math.PI; //m

    public static final double DELIVERY_SPEED_MPS = 20;

    public static final Rotation2d DELIVERY_HOOD_ANGLE = Rotation2d.fromDegrees(20);



    public static final ChassisSpeeds DELIVERY_CHASSIS_SPEEDS = new ChassisSpeeds();

    public static final double XY_DELIVERY_SPEED_TOLERANCE = 0.05;

    public static final double OMEGA_DELIVERY_SPEED_TOLERANCE_RADIANS = Rotation2d.fromDegrees(3).getRadians();



    public static final boolean SHOOT_WITH_MOVEMENT = true;

    public static final double SHOOT_SPEED_MPS_OFFSET_PER_CLICK = 0.5;

    public static final Rotation2d HOOD_ANGLE_OFFSET_PER_CLICK = Rotation2d.fromDegrees(1);

    public static final Rotation2d ROBOT_ANGLE_OFFSET_PER_CLICK = Rotation2d.fromDegrees(1);




    public static final double SHOOTING_SPEED_TOLERANCE = 0.2;

    public static final boolean FLYWHEEL_MOTORS_OPPOSITE = true;



    public static final double ANGULAR_SPEED_DEADBAND_DEGREES = 0.5;
    
    public static final double LINEAR_SPEED_DEADBAND_MPS = 0.3;



    public static final double MIN_DISTANCE_FROM_AZ_TO_SPINUP = 1.67;
}

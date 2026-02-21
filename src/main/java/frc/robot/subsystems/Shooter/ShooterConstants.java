package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShooterConstants {

    public static final Transform3d ROBOT_TO_SHOOTER = 
        new Transform3d(0.3, 0.0, 0.0, Rotation3d.kZero); // Find real translation
    
    public static final double FLYWHEEL_CICUMFRENCE = 0.11 * Math.PI; //m

    public static final double DELIVERY_SPEED_MPS = 20;

    public static final Rotation2d DELIVERY_HOOD_ANGLE = Rotation2d.kZero;

    public static final boolean SHOOT_WITH_MOVEMENT = false;
    
    public static final double SHOOTING_SPEED = 17.5; // m/s

    public static final double SHOOTING_SPEED_TOLERANCE = 0.2;

    public static final boolean FLYWHEEL_MOTORS_OPPOSITE = true;

    public static final double DELIVERY_VELOCITY = 2; // m/s

    public static final double KICKER_PERCENT_OUTPUT = 0.5;


    public static final double ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES = 0.5;
    
    public static final double ZERO_LINEAR_SPEED_TOLERANCE_MPS = 0.005;    
}

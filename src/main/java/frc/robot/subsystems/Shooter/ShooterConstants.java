package frc.robot.subsystems.Shooter;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class ShooterConstants {

    public static final double FLYWHEEL_CICUMFRENCE = 0.11 * Math.PI; //m

    public static final Transform3d ROBOT_TO_SHOOTER = 
        new Transform3d(0.0, 0.0, 0.0, Rotation3d.kZero); // Find real translation
    
    public static final double SHOOTING_SPEED = 17.5; // m/s

    public static final double BASE_SPINUP_SPEED = 100;

    public static final double KICKER_PERCENT_OUTPUT = 0.7;

    public static final double MIN_ACCEL_TO_RESIST = 0;
    
    public static final boolean IS_DEVBOT = true;

    public static final boolean FLYWHEEL_MOTORS_OPPOSITE = true;

    public static final double ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES = 0.5;

    public static final boolean SHOOT_WITH_MOVEMENT = false;

    /* function for adding more to the setpoint for maintaining speed during shooting
    TODO: set actual function
     */
    public static final Function<Double,Double> SHOOTING_SPEED_MODIFIER = (speed) -> 0.0;

    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getLeadShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 27;
        config.motorConfig.motorType = DCMotor.getNeoVortex(2);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Shooting Motor";
        // config.motorConfig.unitConversion = FLYWHEEL_CICUMFRENCE;
    
        config.slot0Config.pidConfig.kP = 0  / FLYWHEEL_CICUMFRENCE;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;   
        config.slot0Config.pidConfig.tolerance = 0.1 * FLYWHEEL_CICUMFRENCE; 
        
        config.slot0Config.feedForwardConfig.velocityFeedforward = 0.10811;
        config.slot0Config.feedForwardConfig.frictionFeedForward = 0.043261;

        config.slot0Config.profileConfig.maximumMeasurementAcceleration = 5; // TODO Decide the optimal number here 
        config.slot0Config.profileConfig.maximumMeasurementVelocity = 5; // TODO Decide the optimal number here

        config.enableVoltageCompensation = false;

        config.slot1Config.feedForwardConfig.velocityFeedforward = 0.10811;
        config.slot1Config.feedForwardConfig.frictionFeedForward = 0.043261;

        config.slot0Config.pidConfig.kP = 0;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;   
        config.slot0Config.pidConfig.tolerance = 1;

        config.simulationConfig.kA = 0.0065362;
        config.simulationConfig.kV = 0.10811;

        config.slot1Config.pidConfig.kP = 0.002;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;   
        config.slot1Config.pidConfig.tolerance = 0.5;


        config.constraintsConfig.minOutput = 0;

        

        return config;
    }

    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getFollowShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 25;
        config.motorConfig.name = "Shooting Motor follower";

        return config;
    }

    /**
     * 
     * @return the motor config for hood motor
     */
    public static BasicSparkConfig getHoodMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 27;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Hood Motor";

        config.slot0Config.pidConfig.kP = 1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }

    /**
     * 
     * @return the motor config for kicker motor
     */
    public static BasicSparkConfig getKickerMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 27;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Kicker Motor";

        config.slot0Config.pidConfig.kP = 1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }


    
}

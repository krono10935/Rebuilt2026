package frc.robot.subsystems.Shooter;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig.AbsoluteEncoderConfig.AbsoluteEncoderRange;

public class ShooterConstants {

    public static final double FLYWHEEL_CICUMFRENCE = 0.11 * Math.PI; //m

    public static final Transform3d ROBOT_TO_SHOOTER = 
        new Transform3d(0.3, 0.0, 0.0, Rotation3d.kZero); // Find real translation
    
    public static final double SHOOTING_SPEED = 17.5; // m/s

    public static final double BASE_SPINUP_SPEED = 10;

    public static final double KICKER_PERCENT_OUTPUT = 0.7;

    public static final double MIN_ACCEL_TO_RESIST = 0;
    
    public static final boolean IS_DEVBOT = true;

    public static final boolean FLYWHEEL_MOTORS_OPPOSITE = true;

    public static final double ZERO_ANGULAR_SPEED_TOLERANCE_DEGREES = 0.5;

    public static final boolean IS_ABSOLUTE_ENCODER_INVERTED = false;

    public static final double ENCODER_ZERO_OFFSET = 0;

    public static final double MOTOR_TO_ENCODER_RATIO = 1.0/8;

    public static final AbsoluteEncoderRange ABSOLUTE_ENCODER_RANGE = AbsoluteEncoderRange.HALF_REVOLUTION;

    public static final double ZERO_LINEAR_SPEED_TOLERANCE_MPS = 0.005;

    public static final boolean SHOOT_WITH_MOVEMENT = true;

    public static final double SHOOTING_SPEED_TOLERANCE = 0.1;


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
        config.motorConfig.unitConversion = FLYWHEEL_CICUMFRENCE;
        
        config.slot0Config.feedForwardConfig.velocityFeedforward = 0.10811 / FLYWHEEL_CICUMFRENCE;
        config.slot0Config.feedForwardConfig.frictionFeedForward = 0.043261/ FLYWHEEL_CICUMFRENCE;

        config.slot0Config.profileConfig.maximumMeasurementAcceleration = 5; // TODO Decide the optimal number here 
        config.slot0Config.profileConfig.maximumMeasurementVelocity = 5; // TODO Decide the optimal number here

        config.enableVoltageCompensation = false;

        // config.slot1Config.feedForwardConfig.velocityFeedforward = 0.10811;
        // config.slot1Config.feedForwardConfig.frictionFeedForward = 0.043261;

        config.slot0Config.pidConfig.kP = 0.1 / FLYWHEEL_CICUMFRENCE;
        config.slot0Config.pidConfig.kI = 0.05;  
        config.slot0Config.pidConfig.tolerance = 0;

        double maxIOutPut = 1.5;// VOLTS
        config.slot0Config.pidConfig.iZone = 0.3;
        config.slot0Config.pidConfig.iMaxAccum = maxIOutPut / config.slot0Config.pidConfig.kI;

        config.slot1Config.feedForwardConfig.velocityFeedforward = 0.10811 / FLYWHEEL_CICUMFRENCE;
        config.slot1Config.feedForwardConfig.frictionFeedForward = 0.043261 / FLYWHEEL_CICUMFRENCE;

        config.slot1Config.pidConfig.kI = 0.05;
        config.slot1Config.pidConfig.kD = 16;   
        config.slot1Config.pidConfig.tolerance = 0;

        config.slot1Config.pidConfig.iZone = 0.6;
        config.slot1Config.pidConfig.iMaxAccum = maxIOutPut / config.slot1Config.pidConfig.kI;

        config.simulationConfig.kA = 0.0065362 / FLYWHEEL_CICUMFRENCE;
        config.simulationConfig.kV = 0.10811 / FLYWHEEL_CICUMFRENCE;

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
        config.motorConfig.id = 20;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 0.255;
        config.motorConfig.name = "Hood Motor";

        config.slot0Config.pidConfig.kP = 0.1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;
        config.slot0Config.pidConfig.tolerance = Rotation2d.fromDegrees(1).getRotations();

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

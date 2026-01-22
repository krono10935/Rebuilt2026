package frc.robot.subsystems.Shooter;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class ShooterConstants {

    public static final double SHOOTING_SPEED = 7; // m/s

    public static final double FLYWHEEL_DIAMETER = 0.11; //m

    public static final double KICKER_PERCENT_OUTPUT = 0.7;

    public static final boolean IS_DEVBOT = true;

    public static final boolean FLYWHEEL_MOTORS_OPPOSITE = true;

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
        config.motorConfig.motorType = DCMotor.getNeoVortex(1);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Shooting Motor";
        // config.motorConfig.unitConversion = FLYWHEEL_DIAMETER * Math.PI;
    
        config.slot0Config.pidConfig.kP = 0;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;

        //TODO: set actual calc if MA calcs dont work
        double maxFreeSpeedMetersPerSec = (Rotation2d.fromRadians(config.motorConfig.motorType.freeSpeedRadPerSec).getRotations());
    
        
        config.slot0Config.feedForwardConfig.setpointFeedForward = maxFreeSpeedMetersPerSec == 0 ? 0 : 12 / maxFreeSpeedMetersPerSec  ;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }

    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getFollowShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 25;
        config.motorConfig.motorType = DCMotor.getNeoVortex(1);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Shooting Motor";
        // config.motorConfig.unitConversion = FLYWHEEL_DIAMETER * Math.PI;
    
        config.slot0Config.pidConfig.kP = 0;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;

        //TODO: set actual calc if MA calcs dont work
        double maxFreeSpeedMetersPerSec = (Rotation2d.fromRadians(config.motorConfig.motorType.freeSpeedRadPerSec).getRotations());
    
        
        config.slot0Config.feedForwardConfig.setpointFeedForward = maxFreeSpeedMetersPerSec == 0 ? 0 : 12 / maxFreeSpeedMetersPerSec  ;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

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
        config.slot0Config.feedForwardConfig.simpleFeedForward = 2;

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
        config.slot0Config.feedForwardConfig.simpleFeedForward = 2;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }


    
}

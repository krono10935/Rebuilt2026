package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class ClimbConstants {

    public static final double MIN_DISTANCE_FROM_TOWER_TO_CLOSE_CLIMB = 0.2; //Meters

    public static final double TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB = 1.0; // we think it its the same time

    public static final double MIN_DISTANCE_FROM_TOWER_TO_OPEN_CLIMB = 3.5; //Meters

    public enum ClimbState{
        CLOSED,
        OPEN,
        CLOSING,
        OPENING
    }

    public static BasicMotorConfig getClimbConfig(){
        BasicMotorConfig config = new BasicSparkConfig();

        config.motorConfig.gearRatio = 1; //TODO: idk yet
        config.motorConfig.id = 47;
        config.motorConfig.idleMode = IdleMode.BRAKE;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.name = "climb motor";

        config.slot0Config.pidConfig.kP = 1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;
        // config.slot0Config.pidConfig.tolerance = 0.1;

        var sparkConfig = ((BasicSparkConfig)config);
        sparkConfig.currentLimitConfig.freeSpeedCurrentLimit = 20; 

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;
        config.simulationConfig.momentOfInertia = 67;
        

        return config;
    }

    public static final Rotation2d CLOSED_ANGLE = Rotation2d.kZero; //Rotation2d.fromRotations(0.75)
    public static final Rotation2d OPENED_ANGLE = Rotation2d.kZero;
}

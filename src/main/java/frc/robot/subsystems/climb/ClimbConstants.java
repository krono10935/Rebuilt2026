package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;

public class ClimbConstants {
    
    public enum ClimbState{
        CLOSED,
        OPEN,
        CLOSING,
        OPENING
    }

    public static BasicMotorConfig getClimbConfig(){
        BasicMotorConfig config = new BasicMotorConfig();

        config.motorConfig.gearRatio = 1; //TODO: idk yet
        config.motorConfig.id = 1; //TODO: find out ts
        config.motorConfig.idleMode = IdleMode.BRAKE;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.name = "climb motor";

        config.slot0Config.pidConfig.kP = 1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;
        // config.slot0Config.pidConfig.tolerance = 0.1;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;
        config.simulationConfig.momentOfInertia = 67;
        

        return config;
    }

    public static final Rotation2d CLOSED_ANGLE = Rotation2d.kZero; //Rotation2d.fromRotations(0.75)
    public static final Rotation2d OPENED_ANGLE = Rotation2d.kZero;
}

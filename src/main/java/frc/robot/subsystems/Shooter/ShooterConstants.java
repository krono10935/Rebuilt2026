package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;

public class ShooterConstants {
    public static final BasicMotorConfig SHOOTING_MOTOR_CONFIG = new BasicMotorConfig();

    public static final double SHOOTING_SPEED = 7; // m/s

    public static final double WHEEL_DIAMETER = 0.11; //m
    

    static {
        SHOOTING_MOTOR_CONFIG.motorConfig.id = 27;
        SHOOTING_MOTOR_CONFIG.motorConfig.motorType = DCMotor.getNeoVortex(1);
        SHOOTING_MOTOR_CONFIG.motorConfig.gearRatio = 1;
        SHOOTING_MOTOR_CONFIG.motorConfig.name = "Skebob";
        SHOOTING_MOTOR_CONFIG.motorConfig.unitConversion = WHEEL_DIAMETER * Math.PI;

        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kP = 0;
        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kI = 0;
        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kD = 0;
        SHOOTING_MOTOR_CONFIG.slot0Config.feedForwardConfig.simpleFeedForward = 2;

        SHOOTING_MOTOR_CONFIG.slot1Config.pidConfig.kP = 0.0001;
        SHOOTING_MOTOR_CONFIG.slot1Config.pidConfig.kI = 0;
        SHOOTING_MOTOR_CONFIG.slot1Config.pidConfig.kD = 0;
        SHOOTING_MOTOR_CONFIG.slot1Config.feedForwardConfig.simpleFeedForward = 2;

        SHOOTING_MOTOR_CONFIG.simulationConfig.kA = 0.1;
        SHOOTING_MOTOR_CONFIG.simulationConfig.kV = 0.1;
        SHOOTING_MOTOR_CONFIG.slot1Config.pidConfig.tolerance = 0.1;
    }
}

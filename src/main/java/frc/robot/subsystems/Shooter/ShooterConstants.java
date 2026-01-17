package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public class ShooterConstants {
    public static final BasicMotorConfig SHOOTING_MOTOR_CONFIG = new BasicMotorConfig();

    public static final double SHOOTING_SPEED = 1; // rots/sec

    static {
        SHOOTING_MOTOR_CONFIG.motorConfig.id = 0;
        SHOOTING_MOTOR_CONFIG.motorConfig.motorType = DCMotor.getNeoVortex(1);
        SHOOTING_MOTOR_CONFIG.motorConfig.gearRatio = 1;
        SHOOTING_MOTOR_CONFIG.motorConfig.name = "SKebob";
        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kP = 1;
        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kI = 0;
        SHOOTING_MOTOR_CONFIG.slot0Config.pidConfig.kD = 0;
    }
}

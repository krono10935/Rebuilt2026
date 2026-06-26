package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IsrIntakeConstants {

    public static final Rotation2d OPEN_POS = Rotation2d.kCW_90deg;
    public static final Rotation2d CLOSED_POS = Rotation2d.kZero;

    public static final double ROLLER_DUTYCYCLE = 0.3;

    public static final double ROLLER_RADIUS_METERS = 1;

    private static final double kG = 0;


    public static final BasicMotorConfig isrLeadRollerMotorConfig = new BasicSparkConfig();


    public static final BasicMotorConfig isrFollowRollerMotorConfig = new BasicSparkConfig();
    static {
        isrLeadRollerMotorConfig.motorConfig.name = "Roller Lead";
        isrLeadRollerMotorConfig.motorConfig.id = 22;
        isrLeadRollerMotorConfig.motorConfig.inverted = false;
        isrLeadRollerMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        isrLeadRollerMotorConfig.motorConfig.gearRatio = 29.0/14;
        isrLeadRollerMotorConfig.motorConfig.motorType = DCMotor.getNeoVortex(2);
        isrLeadRollerMotorConfig.motorConfig.unitConversion = 2 * Math.PI * ROLLER_RADIUS_METERS;

        isrLeadRollerMotorConfig.slot0Config.pidConfig.kP = 0;
        isrLeadRollerMotorConfig.slot0Config.pidConfig.kI = 0;
        isrLeadRollerMotorConfig.slot0Config.pidConfig.kD = 0;

        isrLeadRollerMotorConfig.simulationConfig.kA = 0.023275; // Not calculated
        isrLeadRollerMotorConfig.simulationConfig.kV = 0.31938; // Not calculated

        ((BasicSparkConfig)isrFollowRollerMotorConfig).currentLimitConfig.freeSpeedCurrentLimit = 70;
        ((BasicSparkConfig)isrFollowRollerMotorConfig).currentLimitConfig.freeSpeedRPM = 4000;

    }

    static {
        isrFollowRollerMotorConfig.motorConfig.name = "Roller Follower";
        isrFollowRollerMotorConfig.motorConfig.id = 51;

        
        ((BasicSparkConfig)isrFollowRollerMotorConfig).currentLimitConfig.freeSpeedCurrentLimit = 70;
        ((BasicSparkConfig)isrFollowRollerMotorConfig).currentLimitConfig.freeSpeedRPM = 4000;
    }

    public static final BasicMotorConfig isrLeadPositionMotorConfig = new BasicSparkConfig();

    public static final BasicMotorConfig isrFollowPositionMotorConfig = new BasicSparkConfig();

    static {
        isrLeadPositionMotorConfig.motorConfig.name = "Position Motor";
        isrLeadPositionMotorConfig.motorConfig.id = 62;
        isrLeadPositionMotorConfig.motorConfig.inverted = false;
        isrLeadPositionMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        isrLeadPositionMotorConfig.motorConfig.gearRatio = 5;
        isrLeadPositionMotorConfig.motorConfig.motorType = DCMotor.getNEO(2);
        isrLeadPositionMotorConfig.motorConfig.unitConversion = 360;

        isrLeadPositionMotorConfig.slot0Config.pidConfig.kP = 0;
        isrLeadPositionMotorConfig.slot0Config.pidConfig.kI = 0;
        isrLeadPositionMotorConfig.slot0Config.pidConfig.kD = 0;
        isrLeadPositionMotorConfig.slot0Config.feedForwardConfig.customFeedForward =
            (setpoint) -> kG * Math.cos(setpoint * (2 * Math.PI / 360)); // Gravity feedforward

        isrLeadPositionMotorConfig.simulationConfig.kA = 0.023275; // Not calculated
        isrLeadPositionMotorConfig.simulationConfig.kV = 0.31938; // Not calculated


    }

    static {
        isrFollowPositionMotorConfig.motorConfig.name = "Position Follower";
        isrFollowPositionMotorConfig.motorConfig.id = 40;

        
        ((BasicSparkConfig)isrFollowPositionMotorConfig).currentLimitConfig.freeSpeedCurrentLimit = 70;
        ((BasicSparkConfig)isrFollowPositionMotorConfig).currentLimitConfig.freeSpeedRPM = 4000;
    }
}

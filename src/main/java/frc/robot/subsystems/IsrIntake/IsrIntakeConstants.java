package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;

public class IsrIntakeConstants {

    public static final Rotation2d OPEN_POS = Rotation2d.kCW_90deg;
    public static final Rotation2d CLOSED_POS = Rotation2d.kZero;

    public static final double ROLLER_DUTYCYCLE = 0.3;

    public static final double ROLLER_RADIUS_METERS = 1;

    private static final double kG = 0;


    public static final BasicMotorConfig isrRollerMotorConfig = new BasicMotorConfig();
    static {
        isrRollerMotorConfig.motorConfig.name = "Roller Motor";
        isrRollerMotorConfig.motorConfig.id = 1;
        isrRollerMotorConfig.motorConfig.inverted = false;
        isrRollerMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        isrRollerMotorConfig.motorConfig.gearRatio = 1;
        isrRollerMotorConfig.motorConfig.motorType = DCMotor.getNeoVortex(2);
        isrRollerMotorConfig.motorConfig.unitConversion = 2 * Math.PI * ROLLER_RADIUS_METERS;

        isrRollerMotorConfig.slot0Config.pidConfig.kP = 0;
        isrRollerMotorConfig.slot0Config.pidConfig.kI = 0;
        isrRollerMotorConfig.slot0Config.pidConfig.kD = 0;

        isrRollerMotorConfig.simulationConfig.kA = 0.023275; // Not calculated
        isrRollerMotorConfig.simulationConfig.kV = 0.31938; // Not calculated

    }

    public static final BasicMotorConfig isrPositionMotorConfig = new BasicMotorConfig();
    static {
        isrPositionMotorConfig.motorConfig.name = "Position Motor";
        isrPositionMotorConfig.motorConfig.id = 1;
        isrPositionMotorConfig.motorConfig.inverted = false;
        isrPositionMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        isrPositionMotorConfig.motorConfig.gearRatio = 1;
        isrPositionMotorConfig.motorConfig.motorType = DCMotor.getNEO(2);
        isrPositionMotorConfig.motorConfig.unitConversion = 360;

        isrPositionMotorConfig.slot0Config.pidConfig.kP = 0;
        isrPositionMotorConfig.slot0Config.pidConfig.kI = 0;
        isrPositionMotorConfig.slot0Config.pidConfig.kD = 0;
        isrPositionMotorConfig.slot0Config.feedForwardConfig.customFeedForward =
            (setpoint) -> kG * Math.cos(setpoint * (2 * Math.PI / 360)); // Gravity feedforward

        isrPositionMotorConfig.simulationConfig.kA = 0.023275; // Not calculated
        isrPositionMotorConfig.simulationConfig.kV = 0.31938; // Not calculated


    }
}

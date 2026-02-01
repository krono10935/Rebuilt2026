package frc.robot.subsystems.intake;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.SlotConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IntakeConstants {
    public static final double MOTOR_POWER_PRECENT = 0.7;

    public static final int BEAM_BREAK_CHANNEL = 0;
    public static final int LIMIT_SWITCH_CHANNEL = 1; //TODO: find out

    public static final double CLOSE_POSITION = 0;
    public static final double OPEN_POSITION = 0.67;

    public static final double IDLE_POWER = 10;
    public static final double BALL_INTAKE_ENERGY = 10;

    public static final Rotation2d INTAKE_VELOCITY = Rotation2d.fromDegrees(180);

    public static final double PINION_DIAMETER = 1; //TODO: find out, in meters

    public static final double INTAKE_KT = DCMotor.getNEO(1).withReduction(1).KtNMPerAmp;

    public static final BasicMotorConfig intakeMotorConfig = new BasicSparkConfig();
    static{
        intakeMotorConfig.motorConfig.name = "Intake motor";
        intakeMotorConfig.motorConfig.id = 1;
        intakeMotorConfig.motorConfig.inverted = false;
        intakeMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        intakeMotorConfig.motorConfig.gearRatio = 1;
        intakeMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);

        intakeMotorConfig.simulationConfig.kA = 0.1;
        intakeMotorConfig.simulationConfig.kV = 0.1;

        var specifConfig = (BasicSparkConfig)intakeMotorConfig;

        specifConfig.currentLimitConfig.freeSpeedCurrentLimit = 20;
        specifConfig.currentLimitConfig.stallCurrentLimit = 40;

    }

    public static final BasicMotorConfig positionMotorConfig = new BasicSparkConfig();
    static{
        positionMotorConfig.motorConfig.name = "position motor";
        positionMotorConfig.motorConfig.id = 2;
        positionMotorConfig.motorConfig.inverted = false;
        positionMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        positionMotorConfig.motorConfig.gearRatio = 1;
        positionMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);
        positionMotorConfig.motorConfig.unitConversion = Math.PI * PINION_DIAMETER;

        SlotConfig posConfig = positionMotorConfig.slot0Config;
        posConfig.pidConfig.kP = 0;
        posConfig.pidConfig.kI = 0;
        posConfig.pidConfig.kD = 0;

        positionMotorConfig.constraintsConfig.maxValue = 1;
        positionMotorConfig.constraintsConfig.minValue = 0;
        positionMotorConfig.simulationConfig.kA = 0.1;
        positionMotorConfig.simulationConfig.kV = 0.1;

          var specifConfig = (BasicSparkConfig)positionMotorConfig;

        specifConfig.currentLimitConfig.stallCurrentLimit = 40;
    }
}

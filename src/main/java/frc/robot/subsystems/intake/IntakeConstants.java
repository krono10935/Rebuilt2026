package frc.robot.subsystems.intake;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.SlotConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IntakeConstants {

    //the toletrance of the position of the intake
    public static final double POSITION_TOLERANCE = 0.05;

    //the positions of the intake (closed and opend)
    public static final double CLOSE_POSITION = 0;
    public static final double OPEN_POSITION = 0.67;

    
    public static final double IDLE_POWER = 10;
    public static final double BALL_INTAKE_ENERGY = 10;

    public static final Rotation2d INTAKE_VELOCITY = Rotation2d.fromDegrees(180);

    public static final double PINION_DIAMETER = 1; //TODO: find out, in meters

    public static final double INTAKE_KT = DCMotor.getNEO(1).withReduction(1).KtNMPerAmp;

    public static final double INTAKE_POWER_BALL_COUNTER_DEADBAND = 11;

    public static final double TIME_FOR_INTAKE_TO_CLOSE = 4;
    public static final double TIME_FOR_INTAKE_TO_OPEN = 4;

    public static final double TIME_FOR_BALL_TO_BE_INTAKED = 1.5;

    public static final BasicMotorConfig intakeMotorConfig = new BasicSparkConfig();
    static{
        intakeMotorConfig.motorConfig.name = "Intake motor";
        intakeMotorConfig.motorConfig.id = 1;
        intakeMotorConfig.motorConfig.inverted = false;
        intakeMotorConfig.motorConfig.idleMode = IdleMode.BRAKE; 
        intakeMotorConfig.motorConfig.gearRatio = 1;
        intakeMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);

        intakeMotorConfig.slot0Config.pidConfig.kP = 0;
        intakeMotorConfig.slot0Config.pidConfig.kI = 0;
        intakeMotorConfig.slot0Config.pidConfig.kD = 0;
        intakeMotorConfig.slot0Config.pidConfig.tolerance = 0.1;

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
        posConfig.pidConfig.tolerance = 0.1;

        positionMotorConfig.constraintsConfig.maxValue = 1;
        positionMotorConfig.constraintsConfig.minValue = 0;
        positionMotorConfig.simulationConfig.kA = 0.1;
        positionMotorConfig.simulationConfig.kV = 0.1;

          var specifConfig = (BasicSparkConfig)positionMotorConfig;

        specifConfig.currentLimitConfig.stallCurrentLimit = 40;
    }
}

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

    public static final Rotation2d CLOSE_ANGLE = Rotation2d.kZero;
    public static final Rotation2d OPEN_ANGLE = Rotation2d.kZero;

    public static final double IDLE_POWER = 10;
    public static final double BALL_INTAKE_ENERGY = 10;

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

    public static final BasicMotorConfig openCloseMotorConfig = new BasicSparkConfig();
    static{
        openCloseMotorConfig.motorConfig.name = "Open/Close motor";
        openCloseMotorConfig.motorConfig.id = 2;
        openCloseMotorConfig.motorConfig.inverted = false;
        openCloseMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        openCloseMotorConfig.motorConfig.gearRatio = 1;
        openCloseMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);

        SlotConfig posConfig = openCloseMotorConfig.slot0Config;
        posConfig.pidConfig.kP = 0;
        posConfig.pidConfig.kI = 0;
        posConfig.pidConfig.kD = 0;

        openCloseMotorConfig.constraintsConfig.maxValue = 1;
        openCloseMotorConfig.constraintsConfig.minValue = 0;
        openCloseMotorConfig.simulationConfig.kA = 0.1;
        openCloseMotorConfig.simulationConfig.kV = 0.1;

          var specifConfig = (BasicSparkConfig)openCloseMotorConfig;

        specifConfig.currentLimitConfig.stallCurrentLimit = 40;

        


    }

    

}

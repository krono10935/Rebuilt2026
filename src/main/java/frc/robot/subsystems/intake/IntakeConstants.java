package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.SlotConfig;

public class IntakeConstants {
    public static final double MAX_MOTOR_TEMP = 40.0;

    private static final double INTAKE_GEAR_RATIO = 1;
    private static final double OPEN_CLOSE_GEAR_RATIO = 1;

    private static final DCMotor MOTOR_TYPE = DCMotor.getNEO(1);

    public static final double MOTOR_POWER_PRECENT = 70.0;

    public static final int BEAM_BREAK_CHANNEL = 0;

    public static final BasicMotorConfig intakeMotorConfig = new BasicMotorConfig();
    static{
        intakeMotorConfig.motorConfig.name = "Intake motor";
        intakeMotorConfig.motorConfig.id = 1;
        intakeMotorConfig.motorConfig.inverted = false;
        intakeMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        intakeMotorConfig.motorConfig.gearRatio = INTAKE_GEAR_RATIO;
        intakeMotorConfig.motorConfig.motorType = MOTOR_TYPE;

        intakeMotorConfig.simulationConfig.kA = 0.1;
        intakeMotorConfig.simulationConfig.kV = 0.1;


    }

    public static final BasicMotorConfig openCloseMotorConfig = new BasicMotorConfig();
    static{
        openCloseMotorConfig.motorConfig.name = "Open/Close motor";
        openCloseMotorConfig.motorConfig.id = 2;
        openCloseMotorConfig.motorConfig.inverted = false;
        openCloseMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        openCloseMotorConfig.motorConfig.gearRatio = OPEN_CLOSE_GEAR_RATIO;
        openCloseMotorConfig.motorConfig.motorType = MOTOR_TYPE;

        SlotConfig posConfig = openCloseMotorConfig.slot0Config;
        posConfig.pidConfig.kP = 0;
        posConfig.pidConfig.kI = 0;
        posConfig.pidConfig.kD = 0;

        openCloseMotorConfig.constraintsConfig.maxValue = 1;
        openCloseMotorConfig.constraintsConfig.minValue = 0;
        openCloseMotorConfig.simulationConfig.kA = 0.1;
        openCloseMotorConfig.simulationConfig.kV = 0.1;

        


    }

    

}

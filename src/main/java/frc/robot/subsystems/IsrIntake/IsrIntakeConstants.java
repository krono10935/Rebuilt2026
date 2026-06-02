package frc.robot.subsystems.IsrIntake;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;

public class IsrIntakeConstants {




    public static final BasicMotorConfig isrRollerMotorConfig = new BasicMotorConfig();
    static {
        isrRollerMotorConfig.motorConfig.name = "Roller Motor";
        isrRollerMotorConfig.motorConfig.id = 1;
        isrRollerMotorConfig.motorConfig.inverted = false;
        isrRollerMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        isrRollerMotorConfig.motorConfig.gearRatio = 1;
//        rollerMotorConfig.motorConfig.motorType = ;

        isrRollerMotorConfig.slot0Config.pidConfig.kP = 0;
        isrRollerMotorConfig.slot0Config.pidConfig.kI = 0;
        isrRollerMotorConfig.slot0Config.pidConfig.kD = 0;

        isrRollerMotorConfig.simulationConfig.kA = 0.0;
        isrRollerMotorConfig.simulationConfig.kV = 0.0;

    }

    public static final BasicMotorConfig isrPositionMotorConfig = new BasicMotorConfig();
    static {
        isrPositionMotorConfig.motorConfig.name = "Position Motor";
        isrPositionMotorConfig.motorConfig.id = 1;
        isrPositionMotorConfig.motorConfig.inverted = false;
        isrPositionMotorConfig.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        isrPositionMotorConfig.motorConfig.gearRatio = 1;

        isrPositionMotorConfig.slot0Config.pidConfig.kP = 0;
        isrPositionMotorConfig.slot0Config.pidConfig.kI = 0;
        isrPositionMotorConfig.slot0Config.pidConfig.kD = 0;

        isrPositionMotorConfig.simulationConfig.kA = 0.0;
        isrPositionMotorConfig.simulationConfig.kV = 0.0;


    }
}

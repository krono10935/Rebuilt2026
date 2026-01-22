package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IndexerConstants {


    public static final double SPINNING_PRECENT_OUTPUT = 0.5;

    public static BasicSparkConfig getLeftMotorConfig() {
        BasicSparkConfig configLeftMotor = new BasicSparkConfig();

        configLeftMotor.motorConfig.id = 30;
        configLeftMotor.motorConfig.name = "Left indexer motor";
        configLeftMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configLeftMotor.motorConfig.gearRatio = 1.0;

        configLeftMotor.simulationConfig.kA = 0.2;
        configLeftMotor.simulationConfig.kV = 0.2;


        configLeftMotor.slot0Config.pidConfig.kP = 1;

        return configLeftMotor;
    }

    public static BasicSparkConfig getRightMotorConfig(){
        BasicSparkConfig configRightMotor = new BasicSparkConfig();

        configRightMotor.motorConfig.id = 31;
        configRightMotor.motorConfig.name = "Right indexer motor";
        configRightMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configRightMotor.motorConfig.gearRatio = 1.0;

        configRightMotor.simulationConfig.kA = 0.2;
        configRightMotor.simulationConfig.kV = 0.2;

        configRightMotor.slot0Config.pidConfig.kP = 1;

        return configRightMotor;
    }

}

package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IndexerConstants {


    public static final double SPINNING_TARGET_VELOCITY = 15;//RPS


    public static BasicSparkConfig getLeftMotorConfig() {
        BasicSparkConfig configLeftMotor = new BasicSparkConfig();

        configLeftMotor.motorConfig.id = 40;
        configLeftMotor.motorConfig.name = "Left indexer motor";
        configLeftMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configLeftMotor.motorConfig.gearRatio = 42.0/15;

        configLeftMotor.currentLimitConfig.freeSpeedCurrentLimit = 25;


        configLeftMotor.simulationConfig.kA = 0.2;
        configLeftMotor.simulationConfig.kV = 0.2;


        configLeftMotor.slot0Config.pidConfig.kP = 0.5;

        return configLeftMotor;
    }

    public static BasicSparkConfig getRightMotorConfig(){
        BasicSparkConfig configRightMotor = new BasicSparkConfig();

        configRightMotor.motorConfig.id = 62;
        configRightMotor.motorConfig.name = "Right indexer motor";
        configRightMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configRightMotor.motorConfig.gearRatio = 42.0/15;
        configRightMotor.motorConfig.inverted = true;

        configRightMotor.currentLimitConfig.freeSpeedCurrentLimit = 25;

        configRightMotor.simulationConfig.kA = 0.2;
        configRightMotor.simulationConfig.kV = 0.2;


        configRightMotor.slot0Config.pidConfig.kP = 0.5;

        return configRightMotor;
    }

}

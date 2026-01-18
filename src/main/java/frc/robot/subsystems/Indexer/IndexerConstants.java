package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IndexerConstants {

    public static BasicSparkConfig getConfig() {
        BasicSparkConfig config = new BasicSparkConfig();

        config.motorConfig.id = 30;
        config.motorConfig.name = "Indexer motor";
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 1.0;

        config.simulationConfig.kA = 0.2;
        config.simulationConfig.kV = 0.2;

        config.slot0Config.pidConfig.kP = 1;


        return config;
    }

    public static double SPINNING_PERCENT_OUTPUT = 50;
}

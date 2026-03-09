package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IndexerConstants {


    public static final double SPINNING_TARGET_VELOCITY = 14;//RPS (tuff btw)

    public static final double SPEED_DEADBAND = 0.01;


    public static BasicSparkConfig getLeftMotorConfig() {
        BasicSparkConfig configLeftMotor = new BasicSparkConfig();

        configLeftMotor.motorConfig.id = 40;
        configLeftMotor.motorConfig.name = "Left indexer motor";
        configLeftMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configLeftMotor.motorConfig.gearRatio = 42.0/15;

        configLeftMotor.currentLimitConfig.freeSpeedCurrentLimit = 40;

        configLeftMotor.slot0Config.pidConfig.kP = 0.05;
        configLeftMotor.slot0Config.pidConfig.kI = 0.001;

        configLeftMotor.slot0Config.pidConfig.iMaxAccum = 2;
        configLeftMotor.slot0Config.pidConfig.iZone = 3;

        configLeftMotor.slot0Config.feedForwardConfig.velocityFeedforward = 0.37;

        configLeftMotor.slot0Config.pidConfig.tolerance = 0.2;

        configLeftMotor.simulationConfig.kA = 0.2;
        configLeftMotor.simulationConfig.kV = 0.37;

        configLeftMotor.slot0Config.profileConfig.maximumMeasurementAcceleration = 200;
        configLeftMotor.slot0Config.profileConfig.maximumMeasurementVelocity = 200;

        return configLeftMotor;
    }

    public static BasicSparkConfig getRightMotorConfig(){
        BasicSparkConfig configRightMotor = new BasicSparkConfig();

        configRightMotor.motorConfig.id = 62;
        configRightMotor.motorConfig.name = "Right indexer motor";
        configRightMotor.motorConfig.motorType = DCMotor.getNEO(1);
        configRightMotor.motorConfig.gearRatio = 42.0/15;
        configRightMotor.motorConfig.inverted = true;

        configRightMotor.currentLimitConfig.freeSpeedCurrentLimit = 40;

        configRightMotor.slot0Config.pidConfig.kP = 0.05;
        configRightMotor.slot0Config.pidConfig.kI = 0.001;

        configRightMotor.slot0Config.pidConfig.iMaxAccum = 2;
        configRightMotor.slot0Config.pidConfig.iZone = 3;

        configRightMotor.slot0Config.feedForwardConfig.velocityFeedforward = 0.37;

        configRightMotor.slot0Config.pidConfig.tolerance = 0.2;

        configRightMotor.simulationConfig.kA = 0.2;
        configRightMotor.simulationConfig.kV = 0.37;

        configRightMotor.slot0Config.profileConfig.maximumMeasurementAcceleration = 200;
        configRightMotor.slot0Config.profileConfig.maximumMeasurementVelocity = 200;

        return configRightMotor;
    }

}

package frc.robot.subsystems.drivetrain.module.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CommonModuleConstants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.ModuleConstants;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.FeedForwardsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public enum SwerveModulesMK5 {

    FRONT_LEFT(
            6,  0.067, 12
            ,
            new PIDGains(3, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.2564, 0.22283),
            0.43556,
            4,
            new PIDGains(35, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.6642, 0.31235),
            2.6642,0.28044,
            new Translation2d(0.3, 0.3)),


    FRONT_RIGHT(
            9,  0.052, 13
            ,
            new PIDGains(3, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.252, 0.24746),
            0.43111,
            5,
            new PIDGains(35, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.727, 0.37299),
            2.727,0.47972,
            new Translation2d(0.3, -0.3)),

    BACK_LEFT(
            7,  -0.24, 11
            ,
            new PIDGains(3, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.2663, 0.20489),
            0.48042,
            3,
            new PIDGains(35, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.6894, 0.41461),
            2.6894,0.56055,
            new Translation2d(-0.3, 0.3)),


    BACK_RIGHT(
            8,  -0.25, 10
            ,
            new PIDGains(3, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.2667, 0.21765),
            0.42812,
            2,
            new PIDGains(35, 0, 0, 0, 0, 0),
            new FeedForwardsGains(2.6566, 0.37504),
            2.6566,0.47171,
            new Translation2d(-0.3, 0.3)),;


    SwerveModulesMK5(int canCoderID,
                     double zeroOffset,
                     int driveMotorID,
                     PIDGains drivePIDGains,
                     FeedForwardsGains driveFeedForwards,
                     double driveKA,
                     int steerMotorID,
                     PIDGains steerPIDGains,
                     FeedForwardsGains steerFeedForwards,
                     double steerKV,
                     double steerKA,
                     Translation2d location) {
        BasicTalonFXConfig driveConfig = getGenericConf().DRIVE_CONFIG().copy();
        BasicTalonFXConfig steerConfig = getGenericConf().STEER_CONFIG().copy();

        driveConfig.motorConfig.id = driveMotorID;
        steerConfig.motorConfig.id = steerMotorID;

        driveConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(drivePIDGains);
        steerConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(steerPIDGains);

        driveConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(driveFeedForwards);
        steerConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(steerFeedForwards);

        driveConfig.simulationConfig.kA = driveKA;

        steerConfig.simulationConfig.kV = steerKV;
        steerConfig.simulationConfig.kA = steerKA;

        driveConfig.motorConfig.name = this.name() + " drive motor";
        steerConfig.motorConfig.name = this.name() + " steer motor";

        constants = new ModuleConstants(canCoderID, zeroOffset, driveConfig, steerConfig, location,this.name());

    }

    public final ModuleConstants constants;

    private static CommonModuleConstants genericConf;

    /**
     *
     * @return the generic config
     */
    public static CommonModuleConstants getGenericConf(){
        if(genericConf != null) return genericConf;

        var driveConfig = new BasicTalonFXConfig();

        driveConfig.motorConfig.gearRatio = 6.03;
        driveConfig.motorConfig.unitConversion = 2 * Math.PI * 0.0508;
        driveConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        driveConfig.motorConfig.motorType = DCMotor.getKrakenX60(1);

        driveConfig.currentLimitConfig.statorCurrentLimit = 90;
        driveConfig.currentLimitConfig.supplyCurrentLimit = 0;

        driveConfig.enableFOC = true;

        var steerConfig = new BasicTalonFXConfig();

        steerConfig.enableFOC = true;

        steerConfig.motorConfig.gearRatio = 26.1;
        steerConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        steerConfig.motorConfig.motorType = DCMotor.getKrakenX44(1);

        steerConfig.currentLimitConfig.statorCurrentLimit = 35;

        steerConfig.constraintsConfig.constraintType = ConstraintsGains.ConstraintType.CONTINUOUS;
        steerConfig.constraintsConfig.maxValue = 0.5;
        steerConfig.constraintsConfig.minValue = -0.5;


        genericConf = new CommonModuleConstants(driveConfig,steerConfig,1);
        return genericConf;
    }

    public static ModuleConstants[] getConstants(){
        ModuleConstants[] constants = new ModuleConstants[values().length];
        for(int i=0;i<values().length;i++){
            constants[i] = values()[i].constants;
        }
        return constants;
    }
}

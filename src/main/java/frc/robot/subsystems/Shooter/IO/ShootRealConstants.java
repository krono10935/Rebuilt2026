// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.Shooter.ShooterConstants;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains.ConstraintType;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

/** Add your docs here. */
public class ShootRealConstants {
    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getLeadShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 27;
        config.motorConfig.motorType = DCMotor.getNeoVortex(2);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Shooting Motor";
        config.motorConfig.unitConversion = ShooterConstants.FLYWHEEL_CICUMFRENCE;
        config.enableVoltageCompensation = false;


        //SPIN UP CONSANTS

        config.slot0Config.feedForwardConfig.velocityFeedforward = 0.3128409;
        config.slot0Config.feedForwardConfig.frictionFeedForward = 0.12518549;

        config.slot0Config.pidConfig.kP = 0.5;
        config.slot0Config.pidConfig.kI = 0.05;  
        config.slot0Config.pidConfig.tolerance = 0;

        config.slot0Config.pidConfig.iZone = 1;
        config.slot0Config.pidConfig.iMaxAccum = 1;

        config.slot0Config.profileConfig.maximumMeasurementAcceleration = 5;
        config.slot0Config.profileConfig.maximumMeasurementVelocity = 5;


        //KEEP VELOCITY CONSTANTS

        config.slot1Config.feedForwardConfig.velocityFeedforward = 0.3128409;
        config.slot1Config.feedForwardConfig.frictionFeedForward = 0.12518549;

        config.slot1Config.pidConfig.kI = 0.001;
        config.slot1Config.pidConfig.kD = 16;   
        config.slot1Config.pidConfig.tolerance = 0;

        config.slot1Config.pidConfig.iZone = 1;
        config.slot1Config.pidConfig.iMaxAccum = 1;

        config.simulationConfig.kA = 0.0188913;
        config.simulationConfig.kV = 0.3128409;

        config.constraintsConfig.minOutput = 0;

        return config;
    }

    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getFollowShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 25;
        config.motorConfig.name = "Shooting Motor follower";

        return config;
    }

    /**
     * 
     * @return the motor config for hood motor
     */
    public static BasicSparkConfig getHoodMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 20;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 9;
        config.motorConfig.name = "Hood Motor";
        config.motorConfig.inverted = true;

        config.slot0Config.pidConfig.kP = 0.1;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;
        config.slot0Config.pidConfig.tolerance = Rotation2d.fromDegrees(0.5).getRotations();

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        config.absoluteEncoderConfig.useAbsoluteEncoder = true;

        config.motorConfig.idleMode = IdleMode.BRAKE;
        config.absoluteEncoderConfig.inverted = true;
        config.absoluteEncoderConfig.sensorToMotorRatio = 9;
        config.absoluteEncoderConfig.mechanismToSensorRatio = 80/20;
        config.absoluteEncoderConfig.zeroOffset = Rotation2d.fromDegrees(360 - 0.072 - 60).getRotations() ;

        config.constraintsConfig.maxValue = 0.3;
        config.constraintsConfig.minValue = 0.0;
        config.constraintsConfig.constraintType = ConstraintType.LIMITED;

        config.constraintsConfig.maxOutput = 3;
        config.constraintsConfig.minOutput = 3;

        
        return config;
    }

    /**
     * 
     * @return the motor config for kicker motor
     */
    public static BasicSparkConfig getKickerMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 47;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Kicker Motor";

        config.slot0Config.pidConfig.kP = 0;
        config.slot0Config.pidConfig.kI = 0;
        config.slot0Config.pidConfig.kD = 0;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }
}

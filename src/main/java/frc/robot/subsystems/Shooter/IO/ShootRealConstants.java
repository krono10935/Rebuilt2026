// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.Shooter.ShooterConstants;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains.ConstraintType;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

/** Add your docs here. */
public class ShootRealConstants {
    public static int DUTY_CYCLE_ENCODER_PORT = 1;
    public static double DUTY_CYCLE_ENCODER_ZERO_OFFSET = 0.129;

    public static double KICKER_SPEED_MPS = 4;
    public static double KICKER_MAX_ERROR_FOR_FLYWHEEL_FEEDFORWARD = 0.2;
    public static double KICKER_MIN_ERROR_FOR_FLYWHEEL_FEEDFORWARD = 0.05;
    public static double KICKER_ERROR_FEEDFORWARD_SCALAR = 2;


    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getLeadShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 50;
        config.motorConfig.motorType = DCMotor.getNeoVortex(2);
        config.motorConfig.gearRatio = 1;
        config.motorConfig.name = "Shooting Motor";
        config.motorConfig.unitConversion = ShooterConstants.FLYWHEEL_CICUMFRENCE;
        config.motorConfig.inverted = true;
        config.enableVoltageCompensation = false;


        //SPIN UP CONSANTS
        config.currentLimitConfig.freeSpeedCurrentLimit = 30;
        config.currentLimitConfig.stallCurrentLimit = 80;
        config.currentLimitConfig.freeSpeedRPM = 4000;

        config.slot0Config.feedForwardConfig.velocityFeedforward = 0.31938;
        config.slot0Config.feedForwardConfig.frictionFeedForward = 0.074823;

        config.slot0Config.pidConfig.kP = 0.1;
        config.slot0Config.pidConfig.kI = 0.001;  
        config.slot0Config.pidConfig.tolerance = 0;

        config.enableVoltageCompensation = false;

        config.slot0Config.pidConfig.iZone = 0.3;
        config.slot0Config.pidConfig.iMaxAccum = 2;

        config.slot0Config.profileConfig.maximumMeasurementAcceleration = 10;
        config.slot0Config.profileConfig.maximumMeasurementVelocity = 20;



        //KEEP VELOCITY CONSTANTS

        config.slot1Config.feedForwardConfig.velocityFeedforward = 0.31938;
        config.slot1Config.feedForwardConfig.frictionFeedForward = 0.074823;

        config.slot1Config.pidConfig.kP = 0.6;
        config.slot1Config.pidConfig.kI = 0.001;
        config.slot1Config.pidConfig.kD = 6;   
        config.slot1Config.pidConfig.tolerance = 0;

        config.slot1Config.pidConfig.iZone = 0.6;
        config.slot1Config.pidConfig.iMaxAccum = 2;

        config.simulationConfig.kA = 0.023275;
        config.simulationConfig.kV = 0.31938;

        config.constraintsConfig.minOutput = 0;

        return config;
    }

    /**
     * 
     * @return the motor config for shooting motor
     */
    public static BasicSparkConfig getFollowShootingMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 51;
        config.motorConfig.name = "Shooting Motor follower";

        config.currentLimitConfig.freeSpeedCurrentLimit = 70;
        config.currentLimitConfig.freeSpeedRPM = 4000;

        return config;
    }

    /**
     * 
     * @return the motor config for hood motor
     */
    public static BasicSparkConfig getHoodMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();
        config.motorConfig.id = 19;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 40;
        config.motorConfig.name = "Hood Motor";
        // config.motorConfig.inverted = true;

        config.currentLimitConfig.freeSpeedCurrentLimit = 15;

        config.slot0Config.pidConfig.kP = 40;
        config.slot0Config.pidConfig.kI = 5;
        config.slot0Config.pidConfig.kD = 0;
        config.slot0Config.pidConfig.tolerance = Rotation2d.fromDegrees(0.5
        ).getRotations();

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        // config.absoluteEncoderConfig.useAbsoluteEncoder = false;

        // config.motorConfig.idleMode = IdleMode.BRAKE;
        // config.absoluteEncoderConfig.sensorToMotorRatio = 5;
        // config.absoluteEncoderConfig.mechanismToSensorRatio = 8;
        // config.absoluteEncoderConfig.zeroOffset = 0.16;

        config.constraintsConfig.constraintType = ConstraintType.LIMITED;

        config.constraintsConfig.maxOutput = 3;
        config.constraintsConfig.minOutput = 3;

        config.constraintsConfig.maxValue = 0.08333;
        config.constraintsConfig.minValue = Rotation2d.fromDegrees(0.5).getRotations();
        config.constraintsConfig.constraintType = ConstraintType.LIMITED;

        
        return config;
    }

    /**
     * 
     * @return the motor config for kicker motor
     */
    public static BasicSparkConfig getKickerMotorConfig(){

        final BasicSparkConfig config = new BasicSparkConfig();

        config.motorConfig.id = 30;
        config.motorConfig.motorType = DCMotor.getNEO(1);
        config.motorConfig.gearRatio = 3;
        config.motorConfig.name = "Kicker Motor";
        config.motorConfig.unitConversion = ShooterConstants.FLYWHEEL_CICUMFRENCE / 2;

        config.currentLimitConfig.freeSpeedCurrentLimit = 20;
        config.currentLimitConfig.freeSpeedRPM = 3000;

        config.slot0Config.pidConfig.kP = 0.1;
        config.slot0Config.pidConfig.kI = 0.05;
        config.slot0Config.pidConfig.kD = 0;
        config.slot0Config.feedForwardConfig.velocityFeedforward = 0.382 / (ShooterConstants.FLYWHEEL_CICUMFRENCE / 2);
        config.slot0Config.pidConfig.tolerance = 0.5;
        config.slot0Config.pidConfig.iZone = 2;
        config.slot0Config.pidConfig.iMaxAccum = 3;

        config.simulationConfig.kA = 0.1;
        config.simulationConfig.kV = 0.1;

        return config;
    }
}

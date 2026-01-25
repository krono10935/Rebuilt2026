/*
 * 
 * 
 * 
 * 
 * DO NOT TOUCH THIS BLACK MAGIC
 * 
 * 
 * 
 * 
 */




package frc.robot.subsystems.Shooter.IO;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIONonBasicMotor implements ShooterIO {
    private final SparkFlex leadShootingMotor;
    private final SparkFlex followShootingMotor;

    private ShooterInputs lastInputs;

    private boolean isKickerActive;
    private double targetVelocity;
    public ShooterIONonBasicMotor(){

        var leadConfig = ShooterConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new SparkFlex(leadConfig.motorConfig.id, MotorType.kBrushless);
        followShootingMotor = new SparkFlex(ShooterConstants.getFollowShootingMotorConfig().motorConfig.id, MotorType.kBrushless);
        
        SparkFlexConfig leadFlexConfig = new SparkFlexConfig();
        leadFlexConfig.encoder.velocityConversionFactor(1.0 / 60);
        // leadFlexConfig.closedLoop.feedForward.kA(leadConfig.simulationConfig.kA);
        leadFlexConfig.closedLoop.feedForward.kV(leadConfig.simulationConfig.kV);
        // leadFlexConfig.closedLoop.feedForward.kS(leadConfig.slot0Config.feedForwardConfig.frictionFeedForward);

        // leadFlexConfig.closedLoop.pid(leadConfig.slot0Config.pidConfig.kP,
        // leadConfig.slot0Config.pidConfig.kI,
        // leadConfig.slot0Config.pidConfig.kD);

        // leadFlexConfig.closedLoop.allowedClosedLoopError(leadConfig.slot0Config.pidConfig.tolerance, ClosedLoopSlot.kSlot0);

        leadShootingMotor.configure(leadFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkBaseConfig followFlexConfig = new SparkFlexConfig().follow(leadConfig.motorConfig.id, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);

        // followShootingMotor.configure(followFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        isKickerActive = false;
    }

    @Override
    public void spinUp(double speedMPS){
        double speedRPS = speedMPS / ShooterConstants.FLYWHEEL_CICUMFRENCE;
        targetVelocity = speedRPS;
        leadShootingMotor.getClosedLoopController().setSetpoint(speedRPS, ControlType.kVelocity);
    }

    @Override
    public void keepVelocity(){
        // double kA = 0;
        // double accel = lastInputs.acceleration;
        // if(accel < 0){
        //     kA = -accel * leadConfig.simulationConfig.kA;
        // }
        leadShootingMotor.getClosedLoopController().setSetpoint(targetVelocity , ControlType.kVelocity);
    }


    public void update(ShooterInputs inputs){
        inputs.hoodAngle = Rotation2d.kZero;
        inputs.isKickerActive = false;

        inputs.shooterSpeed = leadShootingMotor.getEncoder().getVelocity();
    }

    public void logSysID(){
        Logger.recordOutput("Shooter sysID/velocity", leadShootingMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter sysID/position", leadShootingMotor.getEncoder().getPosition());
        Logger.recordOutput("Shooter sysID/voltage output",
         leadShootingMotor.getBusVoltage() * leadShootingMotor.getAppliedOutput()
        );

    }

    @Override
    public void stopFlyWheel() {
        leadShootingMotor.stopMotor();
    }

    @Override
    public boolean isShooterAtGoal() {
        return leadShootingMotor.getClosedLoopController().isAtSetpoint();
    }

    @Override
    public void setFlyWheelVoltage(double voltage) {
        leadShootingMotor.setVoltage(voltage);
    }

    @Override
    public void toggleKicker(boolean isActive) {
        // TODO Auto-generated method stub
    }

    @Override
    public void setHoodAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean isHoodAtSetpoint() {
        // TODO Auto-generated method stub
        return true;
    }

    
}

package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBotStrong implements ShooterIO {

    private final BasicMotor leadShootingMotor;
    private final BasicMotor followShootingMotor;

    private final BasicMotorConfig leadConfig;

    private boolean isKickerActive;
    private double targetVelocity;

    public ShooterIODevBotStrong(){
        leadConfig = ShooterConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShooterConstants.getFollowShootingMotorConfig());
        
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);
        

        isKickerActive = false;

    }

    @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
    }

    @Override
    public void keepVelocity(){
        double kA = 0;
        double accel = leadShootingMotor.getMeasurement().acceleration();
        if(accel < ShooterConstants.MIN_ACCEL_TO_RESIST){
            kA = -accel * leadConfig.simulationConfig.kA;
        }
        leadShootingMotor.setControl(targetVelocity , ControlMode.VELOCITY, kA, 1);
    }

    @Override
    public void stopFlyWheel(){
        leadShootingMotor.stop();
    }

    @Override
    public boolean isShooterAtGoal(){
        return leadShootingMotor.atGoal();
    }

    public void setFlyWheelVoltage(double voltage){
        leadShootingMotor.setVoltage(voltage);
        
    }

    @Override
    public void toggleKicker(boolean isActive){}

    @Override
    public void setHoodAngle(Rotation2d angle){}

    @Override
    public boolean isHoodAtSetpoint(){
        return false;
    }

    @Override
    public void update(ShooterInputs inputs){

        inputs.hoodAngle = Rotation2d.kZero;

        inputs.isKickerActive = this.isKickerActive;

        inputs.shooterSpeed = leadShootingMotor.getVelocity();

        SmartDashboard.putData(leadShootingMotor.getController());
        
    }   
}

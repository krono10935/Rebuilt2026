package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBotStrong implements ShooterIO {

    private final BasicMotor leadShootingMotor;
    private final BasicMotor followShootingMotor;

    private boolean isKickerActive;

    public ShooterIODevBotStrong(){

        leadShootingMotor = new BasicSparkFlex(ShooterConstants.getLeadShootingMotorConfig());
        followShootingMotor = new BasicSparkFlex(ShooterConstants.getFollowShootingMotorConfig());
        
        followShootingMotor.followMotor(followShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);
        

        isKickerActive = false;

    }

    @Override
    public void spinUp(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY);
    }

    @Override
    public void stopFlyWheel(){
        leadShootingMotor.stop();
    }

    @Override
    public boolean isShooterAtSetpoint(){
        return leadShootingMotor.atSetpoint();
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

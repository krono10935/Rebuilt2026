package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBot implements ShooterIO {

    private final BasicSparkFlex shootingMotor;

    private boolean isKickerActive;

    public ShooterIODevBot(){

        shootingMotor = new BasicSparkFlex(ShooterConstants.getShootingMotorConfig());

        isKickerActive = false;

    }

    @Override
    public void shoot(double speedMPS){
        shootingMotor.setControl(speedMPS , ControlMode.VELOCITY, ShooterConstants.SHOOTING_PID_SLOT);
    }

    @Override
    public void spinUp(double speedMPS){
        shootingMotor.setControl(speedMPS , ControlMode.VELOCITY, ShooterConstants.SPIN_UP_PID_SLOT);
    }

    @Override
    public void stopFlyWheel(){
        shootingMotor.stop();
    }

    @Override
    public boolean isShooterAtSetpoint(){
        return shootingMotor.atSetpoint();
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

        inputs.shooterSpeed = shootingMotor.getVelocity();

        SmartDashboard.putData(shootingMotor.getController());
        
    }   
}

package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBot implements ShooterIO {

    private final BasicSparkFlex shootingMotor;

    private final BasicMotorConfig config;

    private boolean isKickerActive;

    public ShooterIODevBot(){
        config = ShooterConstants.getLeadShootingMotorConfig();

        shootingMotor = new BasicSparkFlex(config);

        isKickerActive = false;
        
        SmartDashboard.putData(shootingMotor.getController());

    }

    @Override
    public void spinUp(double speedMPS){
        shootingMotor.setControl(speedMPS , ControlMode.VELOCITY);
    }

    @Override
    public void stopFlyWheel(){
        shootingMotor.stop();
    }

    @Override
    public boolean isShooterAtSetpoint(){
        return shootingMotor.atSetpoint();
    }

    public void setFlyWheelVoltage(double voltage){
        shootingMotor.setVoltage(voltage);
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
    }   
}

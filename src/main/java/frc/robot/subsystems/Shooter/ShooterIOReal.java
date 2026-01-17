package frc.robot.subsystems.Shooter;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIOReal implements ShooterIO {
    private BasicMotor shootingMotor;

    public ShooterIOReal(){
        shootingMotor = new BasicSparkFlex(ShooterConstants.SHOOTING_MOTOR_CONFIG);
    }

    @Override
    public void shoot(double speed) {
        shootingMotor.setControl(speed, ControlMode.VELOCITY);  
    }

    public void stop(){
        shootingMotor.stop();
    }
    
}

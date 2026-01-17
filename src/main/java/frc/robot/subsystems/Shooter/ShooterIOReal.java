package frc.robot.subsystems.Shooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIOReal implements ShooterIO {
    private BasicMotor shootingMotor;

    public ShooterIOReal(){
        shootingMotor = new BasicSparkFlex(ShooterConstants.SHOOTING_MOTOR_CONFIG);
        SmartDashboard.putData(shootingMotor.getController());
    }

    @Override
    public void shoot(double speed, int pidSlot) {
        shootingMotor.setControl(speed, ControlMode.VELOCITY, pidSlot);  
    }

    public void stop(){
        shootingMotor.stop();
    }

    public void update(){
        
    }
    
}

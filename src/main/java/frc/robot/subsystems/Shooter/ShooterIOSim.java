package frc.robot.subsystems.Shooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class ShooterIOSim implements ShooterIO {
    private BasicMotorSim shootingMotor;

    public ShooterIOSim(){
        shootingMotor = new BasicMotorSim(ShooterConstants.SHOOTING_MOTOR_CONFIG);
    }

    @Override
    public void shoot(double speed, int pidSlot) {
        shootingMotor.setControl(speed, ControlMode.VELOCITY, pidSlot);  
    }

    public void stop(){
        shootingMotor.stop();
    }

    public void update(){
        SmartDashboard.putData(shootingMotor.getController());
    }
    
}

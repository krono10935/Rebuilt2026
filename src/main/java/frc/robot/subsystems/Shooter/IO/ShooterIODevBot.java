package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBot implements ShooterIO {

    private final BasicSparkFlex shootingMotor;

    private final BasicMotorConfig config;

    private boolean isKickerActive;
    private double targetVelocity;

    public ShooterIODevBot(){
        config = ShooterConstants.getLeadShootingMotorConfig();

        shootingMotor = new BasicSparkFlex(config);

        isKickerActive = false;
        
        SmartDashboard.putData(shootingMotor.getController());

    }

     @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        shootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
    }

    @Override
    public void keepVelocity(){
        double kA = 0;
        double accel = shootingMotor.getMeasurement().acceleration();
        if(accel < ShooterConstants.MIN_ACCEL_TO_RESIST){
            kA = -accel * config.simulationConfig.kA;
        }
        shootingMotor.setControl(targetVelocity , ControlMode.VELOCITY, kA, 1);
    }

    @Override
    public void stopFlyWheel(){
        shootingMotor.stop();
    }

    @Override
    public boolean isShooterAtGoal(){
        return shootingMotor.atGoal();
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

    @Override
    public void logSysID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logSysID'");
    }   
}

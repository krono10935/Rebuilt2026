package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final BasicMotorSim shootingMotor;

    private final BasicMotorSim hoodMotor;

    private final BasicMotorSim kickerMotor;

    private boolean isKickerActive;

    public ShooterIOSim(){

        shootingMotor = new BasicMotorSim(ShooterConstants.getShootingMotorConfig());

        hoodMotor =  new BasicMotorSim(ShooterConstants.getHoodMotorConfig());

        kickerMotor =  new BasicMotorSim(ShooterConstants.getKickerMotorConfig());

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
    public void toggleKicker(boolean isActive){
        if(isActive){
            kickerMotor.setPercentOutput(ShooterConstants.KICKER_PERCENT_OUTPUT);
        }
        else{
            kickerMotor.stop();
        }

        isKickerActive = isActive;
    }

    @Override
    public void setHoodAngle(Rotation2d angle){
        hoodMotor.setControl(angle.getRotations(), ControlMode.POSITION);
    }

     @Override
    public boolean isHoodAtSetpoint(){
        return hoodMotor.atSetpoint();
    }

    @Override
    public void update(ShooterInputs inputs){

        inputs.hoodAngle = Rotation2d.fromRotations(hoodMotor.getPosition());

        inputs.isKickerActive = this.isKickerActive;

        inputs.shooterSpeed = shootingMotor.getVelocity();
        
    }   
}

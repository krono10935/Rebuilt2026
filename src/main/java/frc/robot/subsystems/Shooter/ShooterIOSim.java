package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final BasicMotorSim leadShootingMotor;
    private final BasicMotorSim followShootingMotor;

    private final BasicMotorSim hoodMotor;

    private final BasicMotorSim kickerMotor;

    private boolean isKickerActive;

    public ShooterIOSim(){
        leadShootingMotor = new BasicMotorSim(ShooterConstants.getLeadShootingMotorConfig());
        followShootingMotor = new BasicMotorSim(ShooterConstants.getFollowShootingMotorConfig());
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);


        hoodMotor =  new BasicMotorSim(ShooterConstants.getHoodMotorConfig());

        kickerMotor =  new BasicMotorSim(ShooterConstants.getKickerMotorConfig());

        isKickerActive = false;

    }

    @Override
    public void shoot(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY, ShooterConstants.SHOOTING_PID_SLOT);
    }

    @Override
    public void spinUp(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY, ShooterConstants.SPIN_UP_PID_SLOT);
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

        inputs.shooterSpeed = leadShootingMotor.getVelocity();
        
    }   
}

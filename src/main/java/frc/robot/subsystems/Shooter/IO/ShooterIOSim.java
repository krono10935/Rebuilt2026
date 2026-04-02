package frc.robot.subsystems.Shooter.IO;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.flywheel.BasicFlywheelSim;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final BasicMotor leadShootingMotor;
    private final BasicMotor followShootingMotor;

    private final BasicMotor hoodMotor;

    private final BasicMotor kickerMotor;

    private final BasicMotorConfig shooterConfig;

    private double targetVelocity;

    public ShooterIOSim(){
        shooterConfig = ShootRealConstants.getLeadShootingMotorConfig();
        leadShootingMotor = new BasicFlywheelSim(shooterConfig);
        followShootingMotor = new BasicFlywheelSim(ShootRealConstants.getFollowShootingMotorConfig());

        followShootingMotor.followMotor(leadShootingMotor, false); // The default should be not inverted

        hoodMotor =  new BasicMotorSim(ShootRealConstants.getHoodMotorConfig());

        kickerMotor =  new BasicMotorSim(ShootRealConstants.getKickerMotorConfig());

    }

    @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY,0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(targetVelocity , ControlMode.PROFILED_VELOCITY, 1);
        Logger.recordOutput("Shooter/keeping", true);
    }

    @Override
    public void stopFlyWheel(){
        leadShootingMotor.stop();
    }

    private boolean isShooterAtGoal(){
        return Math.abs(leadShootingMotor.getController().getGoalAsDouble() -
         leadShootingMotor.getVelocity())
          <= ShooterConstants.SHOOTING_SPEED_TOLERANCE;
    }

    public void setFlyWheelVoltage(double voltage){
        leadShootingMotor.setVoltage(voltage);
    }

    @Override
    public void toggleKicker(boolean isActive){
        if(isActive){
            kickerMotor.setControl(ShootRealConstants.KICKER_SPEED_MPS, ControlMode.VELOCITY);
        }
        else{
            kickerMotor.stop();
        }

    }

    @Override
    public void setHoodAngle(Rotation2d angle){
        hoodMotor.setControl(angle.getRotations(), ControlMode.POSITION);
    }

    
    private boolean isHoodAtSetpoint(){
        return (Math.abs(hoodMotor.getError()) <= ShootRealConstants.HOOD_TOLERANCE.getRotations());
    }

    @Override
    public void update(ShooterInputs inputs){
        inputs.shooterSpeedMPS = leadShootingMotor.getVelocity();
        inputs.isFlywheelAtGoal = isShooterAtGoal();
        inputs.isKickerStuck = isKickerStuck();
        inputs.isHoodAtSetpoint = isHoodAtSetpoint();
    }

    @Override
    public void logSysID() {
        
    }

    private boolean isKickerStuck() {
        return false;
    } 
}

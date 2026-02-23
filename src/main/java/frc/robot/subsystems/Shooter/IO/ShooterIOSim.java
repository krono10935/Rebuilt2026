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

    private final BasicMotor hoodMotor;

    private final BasicMotor kickerMotor;

    private final BasicMotorConfig shooterConfig;

    private boolean isKickerActive;

    public ShooterIOSim(){
        shooterConfig = ShootRealConstants.getLeadShootingMotorConfig();
        leadShootingMotor = new BasicFlywheelSim(shooterConfig);

        hoodMotor =  new BasicMotorSim(ShootRealConstants.getHoodMotorConfig());

        kickerMotor =  new BasicMotorSim(ShootRealConstants.getKickerMotorConfig());

        isKickerActive = false;

    }

        @Override
    public void spinUp(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){

        double arbFF = 0;
        double kicker_error = kickerMotor.getController().getSetpointAsDouble() - kickerMotor.getVelocity();

        if (ShootRealConstants.KICKER_MAX_ERROR_FOR_FLYWHEEL_FEEDFORWARD < kicker_error 
            && kicker_error > ShootRealConstants.KICKER_MIN_ERROR_FOR_FLYWHEEL_FEEDFORWARD){
            arbFF = kicker_error * ShootRealConstants.KICKER_ERROR_FEEDFORWARD_SCALAR;
        }

        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, arbFF, 1);
        Logger.recordOutput("Shooter/keeping", true);
    }

    @Override
    public void stopFlyWheel(){
        leadShootingMotor.stop();
    }

    @Override
    public boolean isShooterAtGoal(){
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

    @Override
    public void logSysID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logSysID'");
    }   
}

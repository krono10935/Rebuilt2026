package frc.robot.subsystems.Shooter.IO;

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
    private double targetVelocity;

    public ShooterIOSim(){
        shooterConfig = ShooterConstants.getLeadShootingMotorConfig();
        leadShootingMotor = new BasicFlywheelSim(shooterConfig);

        hoodMotor =  new BasicMotorSim(ShooterConstants.getHoodMotorConfig());

        kickerMotor =  new BasicMotorSim(ShooterConstants.getKickerMotorConfig());

        isKickerActive = false;

    }

        @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
    }

    @Override
    public void keepVelocity(){
        double kA = 0;
        double accel = leadShootingMotor.getMeasurement().acceleration();
        if(accel < ShooterConstants.MIN_ACCEL_TO_RESIST){
            kA = -accel * shooterConfig.simulationConfig.kA;
        }
        leadShootingMotor.setControl(targetVelocity , ControlMode.VELOCITY, kA, 1);
    }

    @Override
    public void stopFlyWheel(){
        leadShootingMotor.stop();
    }

    @Override
    public boolean isShooterAtGoal(){
        return leadShootingMotor.atGoal();
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

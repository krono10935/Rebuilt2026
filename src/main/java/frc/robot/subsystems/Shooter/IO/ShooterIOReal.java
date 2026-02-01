package frc.robot.subsystems.Shooter.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class ShooterIOReal implements ShooterIO {

    private final BasicSparkFlex leadShootingMotor;
    private final BasicSparkFlex followShootingMotor;

    private final BasicSparkMAX hoodMotor;

    private final BasicSparkMAX kickerMotor;

    private final BasicMotorConfig leadConfig;

    private boolean isKickerActive;
    private double targetVelocity;

    public ShooterIOReal(){
        leadConfig = ShooterConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShooterConstants.getFollowShootingMotorConfig());
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);


        hoodMotor =  new BasicSparkMAX(ShooterConstants.getHoodMotorConfig());

        kickerMotor =  new BasicSparkMAX(ShooterConstants.getKickerMotorConfig());

        isKickerActive = false;

    }

    @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY,0);
    }

    @Override
    public void keepVelocity(){
        double kA = 0;
        double accel = leadShootingMotor.getMeasurement().acceleration();
        if(accel < ShooterConstants.MIN_ACCEL_TO_RESIST){
            kA = -accel * leadConfig.simulationConfig.kA;
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

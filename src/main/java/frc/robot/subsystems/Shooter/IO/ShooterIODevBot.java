package frc.robot.subsystems.Shooter.IO;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class ShooterIODevBot implements ShooterIO {

    private final BasicMotor leadShootingMotor;
    private final BasicMotor followShootingMotor;

    private final BasicMotor hoodMotor;

    private final BasicMotorConfig leadConfig;

    private boolean isKickerActive;

    public ShooterIODevBot(){
        leadConfig = ShooterConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShooterConstants.getFollowShootingMotorConfig());
        
        leadShootingMotor.getController().setSendableSlot(0);
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);

        hoodMotor = new BasicSparkMAX(ShooterConstants.getHoodMotorConfig());
        ((BasicSparkMAX)hoodMotor).useAbsoluteEncoder(
            ShooterConstants.IS_HOOD_ABSOLUTE_ENCODER_INVERTED,
            ShooterConstants.HOOD_ENCODER_ZERO_OFFSET, 
            ShooterConstants.HOOD_MOTOR_TO_ENCODER_RATIO, 
            ShooterConstants.HOOD_ABSOLUTE_ENCODER_RANGE
        );
        

        isKickerActive = false;
        SmartDashboard.putData(hoodMotor.getController());

    }

    @Override
    public void spinUp(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 1);
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
    public void toggleKicker(boolean isActive){}

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

        inputs.hoodAngle = Rotation2d.kZero;

        inputs.isKickerActive = this.isKickerActive;

        inputs.shooterSpeed = leadShootingMotor.getVelocity();
        
    }

    @Override
    public void logSysID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logSysID'");
    }   
}

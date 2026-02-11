package frc.robot.subsystems.Shooter.IO;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
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

    private final BasicMotor kickerMotor;

    private final BasicMotorConfig leadConfig;

    private boolean isKickerActive;

    private final DoubleSupplier dutyCycleSupplier;

    public ShooterIODevBot(){
        leadConfig = ShooterDevBotConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShooterDevBotConstants.getFollowShootingMotorConfig());
        
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);

        hoodMotor = new BasicSparkMAX(ShooterDevBotConstants.getHoodMotorConfig());

        kickerMotor = new BasicSparkMAX(ShooterDevBotConstants.getKickerMotorConfig());

        isKickerActive = false;
        
        leadShootingMotor.getController().setSendableSlot(1);
        SmartDashboard.putData(leadShootingMotor.getController());


        var motor = ((BasicSparkFlex)leadShootingMotor);


        var spark = motor.getMotor();

        motor.getSparkConfig().signals.appliedOutputPeriodMs(10);

        spark.configure(motor.getSparkConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        dutyCycleSupplier = spark::getAppliedOutput;
    }

    @Override
    public void spinUp(double speedMPS){
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, 0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){
        double output = 0;

        if(!isShooterAtGoal() && leadShootingMotor.getMeasurement().acceleration() < 0 && leadShootingMotor.getVelocity() < speedMPS)
            output = 6;

        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY, output, 1);
        Logger.recordOutput("Shooter/keeping", true);
    }

    @Deprecated
    private double calculateFF(double target){
        if(isShooterAtGoal()) return 0;

        if(leadShootingMotor.getVelocity() > target) return 0;

        if(leadShootingMotor.getMeasurement().acceleration() < 0){
            return 12;
        }
        else{
            double error = Math.abs(target - leadShootingMotor.getVelocity());

            if(error >= 2) return 12;
            else return MathUtil.inverseInterpolate(0, 2, error) * 12;
        }
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
        if (isActive){
            kickerMotor.setPercentOutput(ShooterConstants.KICKER_PERCENT_OUTPUT);
        } else {
            kickerMotor.stop();
        }
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

        inputs.hoodAngle = Rotation2d.kZero;

        inputs.isKickerActive = this.isKickerActive;

        inputs.shooterSpeed = leadShootingMotor.getVelocity();

        Logger.recordOutput("Shooter/motorDutyCycle", dutyCycleSupplier.getAsDouble());
        
    }

    @Override
    public void logSysID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logSysID'");
    }   
}

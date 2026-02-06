package frc.robot.subsystems.Shooter.IO;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

public class ShooterIODevBot implements ShooterIO {

    private final BasicMotor leadShootingMotor;
    private final BasicMotor followShootingMotor;

    private final BasicMotorConfig leadConfig;

    private boolean isKickerActive;

    private LoggedNetworkNumber shiityP;

    public ShooterIODevBot(){
        leadConfig = ShooterConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShooterConstants.getFollowShootingMotorConfig());
        
        leadShootingMotor.getController().setSendableSlot(1);
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);
        

        isKickerActive = false;
        SmartDashboard.putData(leadShootingMotor.getController());

        shiityP = new LoggedNetworkNumber("p");
        shiityP.setDefault(0);
        
    }

    @Override
    public void spinUp(double speedMPS){
        // double targetVelocity = speedMPS / ShooterConstants.FLYWHEEL_CICUMFRENCE;
        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY, 0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){
        // double targetVelocity = speedMPS / ShooterConstants.FLYWHEEL_CICUMFRENCE;

        double shitP = leadShootingMotor.getController().getGoalAsDouble() - leadShootingMotor.getVelocity();

        shitP *= shiityP.getAsDouble();

        leadShootingMotor.setControl(speedMPS , ControlMode.VELOCITY, Math.max(0, shitP), 1);
        Logger.recordOutput("Shooter/keeping", true);
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

        inputs.shooterSpeed = leadShootingMotor.getVelocity();
        
    }

    @Override
    public void logSysID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logSysID'");
    }   
}

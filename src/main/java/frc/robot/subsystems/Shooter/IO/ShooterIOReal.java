package frc.robot.subsystems.Shooter.IO;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    private final DutyCycleEncoder dutyCycleEncoder;

    private boolean isKickerActive;
    private double targetVelocity;

    private final DoubleSupplier leadShooterMotorDutyCycle;

    public ShooterIOReal(){
        leadConfig = ShootRealConstants.getLeadShootingMotorConfig();

        leadShootingMotor = new BasicSparkFlex(leadConfig);
        followShootingMotor = new BasicSparkFlex(ShootRealConstants.getFollowShootingMotorConfig());
        followShootingMotor.followMotor(leadShootingMotor, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE);

        dutyCycleEncoder = new DutyCycleEncoder(ShootRealConstants.DUTY_CYCLE_ENCODER_PORT);
        
        hoodMotor =  new BasicSparkMAX(ShootRealConstants.getHoodMotorConfig());

        kickerMotor =  new BasicSparkMAX(ShootRealConstants.getKickerMotorConfig());

        isKickerActive = false;

        leadShootingMotor.getController().setSendableSlot(1);

        var spark = leadShootingMotor.getMotor();

        leadShooterMotorDutyCycle = spark::getAppliedOutput;

    

        CommandScheduler.getInstance().schedule(new InstantCommand(
                () -> hoodMotor.resetEncoder((dutyCycleEncoder.get() - ShootRealConstants.DUTY_CYCLE_ENCODER_ZERO_OFFSET) / 8))
                        .beforeStarting(new WaitUntilCommand(() -> dutyCycleEncoder.get() != 0)).ignoringDisable(true));
        SmartDashboard.putData(hoodMotor.getController());
    }

    @Override
    public void spinUp(double speedMPS){
        targetVelocity = speedMPS;
        leadShootingMotor.setControl(speedMPS , ControlMode.PROFILED_VELOCITY,0);
        Logger.recordOutput("Shooter/keeping", false);
    }

    @Override
    public void keepVelocity(double speedMPS){ // TODO WTF THIS SHOULD NOT WORK
        leadShootingMotor.setControl(targetVelocity , ControlMode.PROFILED_VELOCITY, 1);
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
            kickerMotor.setControl(ShootRealConstants.KICKER_SPEED_MPS, ControlMode.VELOCITY);
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
        return (Math.abs(hoodMotor.getError()) <= ShootRealConstants.HOOD_TOLERANCE.getRotations());
    }

    @Override
    public void update(ShooterInputs inputs){

        inputs.hoodAngle = Rotation2d.fromRotations(hoodMotor.getPosition());

        inputs.isKickerActive = this.isKickerActive;

        inputs.shooterSpeed = leadShootingMotor.getVelocity();

        Logger.recordOutput("duty cycle", leadShooterMotorDutyCycle.getAsDouble());
        Logger.recordOutput("duty cycle/ encoder", (dutyCycleEncoder.get() - ShootRealConstants.DUTY_CYCLE_ENCODER_ZERO_OFFSET) / 8);
        // hoodMotor.resetEncoder((dutyCycleEncoder.get() - ShootRealConstants.DUTY_CYCLE_ENCODER_ZERO_OFFSET) / 8);

        
    }

    @Override
    public void logSysID() {
        
    }

    @Override
    public boolean isKickerStuck() {
        return Math.abs(kickerMotor.getController().getSetpointAsDouble()) 
        < ShootRealConstants.KICKER_SPEED_DEADBAND;
    } 
}

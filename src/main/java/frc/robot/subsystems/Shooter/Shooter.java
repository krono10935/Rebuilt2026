// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.utils.ErrorMessage;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.IO.ShooterIODevBot;
import frc.robot.subsystems.Shooter.IO.ShooterIOReal;
import frc.robot.subsystems.Shooter.IO.ShooterIOSim;
import frc.robot.subsystems.drivetrain.constants.ChassisType;


public class Shooter extends SubsystemBase implements ErrorMessage.ErrorSender {

    private final ShooterIO io;

    private final Indexer indexer;

    private final ShooterInputsAutoLogged inputs;

    private boolean failedToClose;

    /**
    * Create a shooter IO based on the robot's state (sim, dev, comp)
    */
    public Shooter() {

    indexer = new Indexer();

    if (!RobotBase.isReal()){
      io = new ShooterIOSim();
    }
    else if(Constants.CHASSIS_TYPE == ChassisType.DEVBOT){
      io = new ShooterIODevBot();
    }
    else{
      io = new ShooterIOReal();
    }

    inputs = new ShooterInputsAutoLogged();

    failedToClose = false;

    ErrorMessage.create(this,
            "error closing" + this.getName(),
            () -> failedToClose);

    }


    @Override
    public void periodic(){

    io.update(inputs);

    Logger.processInputs(getName(), inputs);

    Logger.recordOutput("Shooter/current command", getCurrentCommand() == null? "None" : getCurrentCommand().getName());
    Logger.recordOutput("Shooter/is hood at setpoint", isHoodAtSetpoint());
    Logger.recordOutput("Shooter/is shooter at setpoint", isShooterAtGoal());

    }


    public void hasFailedToClose(boolean failedToClose){
    this.failedToClose = failedToClose;
    }

    public void logSysID(){
    io.logSysID();
    }

    public void setCloseMode(boolean isClosing){
      this.failedToClose = isClosing;
    }

    /**
    *
    * @param speedMPS speed to spinUp to
    */
    public void spinUp(double speedMPS){
    io.spinUp(speedMPS);
    }

    public void keepVelocity(double speedMPS){
    io.keepVelocity(speedMPS);
    }

    /**
    * stop the flywheel
    */
    public void stopFlyWheel(){
    io.stopFlyWheel();
    }

    /**
    *
    * @return is shooter at setpoint
    */
    public boolean isShooterAtGoal(){
    return io.isShooterAtGoal();
    }

    /**
    *
    * @param voltage apply the voltage to the flywheel motor(s)
    */
    public void setVoltage(double voltage){
    io.setFlyWheelVoltage(voltage);
    }

    /**
    *
    * @param isActive toggle on or off the kicker
    */
    public void toggleKicker(boolean isActive){
    io.toggleKicker(isActive);
    }

    /**
    *
    * @return is hood at setpoint
    */
    public boolean isHoodAtSetpoint(){
    return io.isHoodAtSetpoint();
    }

    /**
    *
    * @param angle angle to set the hood to
    */
    public void setHoodAngle(Rotation2d angle){
    io.setHoodAngle(angle);
    }

    public boolean readyToShoot(){
    return isHoodAtSetpoint() && isShooterAtGoal();
    }

    public Indexer getIndexer(){
    return indexer;
    }

    public void dutyCycle(double dutyCycle){
      io.setFlyWheelVoltage(dutyCycle * 12);
    }

    @Override
    public void send(boolean shouldDisplayError, int code) {
        failedToClose = shouldDisplayError;
    }
}

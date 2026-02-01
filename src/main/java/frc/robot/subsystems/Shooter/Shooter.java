// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.IO.ShooterIODevBotStrong;
import frc.robot.subsystems.Shooter.IO.ShooterIONonBasicMotor;
import frc.robot.subsystems.Shooter.IO.ShooterIOReal;
import frc.robot.subsystems.Shooter.IO.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  private final ShooterInputsAutoLogged inputs;

  private ShootingParameters shooterParams;

  /**
   * Create a shooter IO based on the robot's state (sim, dev, comp)
   */
  public Shooter() {

    if (!RobotBase.isReal()){
      io = new ShooterIOSim();
    }
    else if(ShooterConstants.IS_DEVBOT){
      io = new ShooterIODevBotStrong();
    }
    else{
      io = new ShooterIOReal();
    }
    // io = new ShooterIONonBasicMotor();

    inputs = new ShooterInputsAutoLogged();

  }
  

  @Override
  public void periodic(){

    io.update(inputs);

    Logger.processInputs(getName(), inputs);

    Logger.recordOutput("Shooter/current command", getCurrentCommand() == null? "None" : getCurrentCommand().getName());
    Logger.recordOutput("Shooter/is hood at setpoint", isHoodAtSetpoint());
    Logger.recordOutput("Shooter/is shooter at setpoint", isShooterAtGoal());

  }

  public void logSysID(){
    io.logSysID();
  }

  public void updateShootingParameters(Drivetrain drivetrain){
    ShotCalculator.getInstance().clearShootingParameters();
    shooterParams = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds(), 
     ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getGyroAngle()));
  }

  public ShootingParameters getShootParameters(){
    return shooterParams;
  }

  /**
   * 
   * @param speedMPS speed to spinUp to
   */
  public void spinUp(double speedMPS){
    io.spinUp(speedMPS);
  }

  public void keepVelocity(){
    io.keepVelocity();
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

  public Command stopShooter(){
      return new InstantCommand(this::stopFlyWheel, this);
  }

}

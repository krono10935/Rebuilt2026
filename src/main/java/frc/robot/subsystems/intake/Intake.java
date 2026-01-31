// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {
    if(Robot.isReal()){
      io = new IntakeIOSpark();
    }
    else{
      io = new IntakeIOSim();
    }

  }


  public void toggleActivationMotor(Rotation2d setActivation){
    io.setActivationMotorPos(setActivation);
  }

  public void startIntake(){
    io.setPercentOutput(IntakeConstants.MOTOR_POWER_PRECENT);
  }

  public void stopMotor(){
    io.stopMotor();
  }

  public double getPower(){
    return inputs.power;
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    String currCommand = getCurrentCommand() == null? "Null" : getCurrentCommand().getName();
    Logger.recordOutput("Intake/Current Command ", currCommand);
  }
}

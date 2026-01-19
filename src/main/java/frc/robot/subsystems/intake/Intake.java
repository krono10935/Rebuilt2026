// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import org.littletonrobotics.junction.Logger;

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

  public void keepPos(){
    io.setPos(io.getPos());
  }

  public void stopMotor(){
    io.stopMotor();
  }

  public boolean isMotorOnFire(){
    return inputs.temp >= IntakeConstants.MAX_MOTOR_TEMP;
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

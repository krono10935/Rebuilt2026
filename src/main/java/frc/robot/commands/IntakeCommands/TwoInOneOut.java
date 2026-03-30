// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class TwoInOneOut extends Command {
    private final Intake intake;
    private final LoggedNetworkNumber distanceToMoveOut;
    private final LoggedNetworkNumber distanceToMoveIn;
    private final LoggedNetworkNumber powerMoveOut;
    private final LoggedNetworkNumber powerToMoveIn;
    private final LoggedNetworkNumber positionTolerance;
    private double setpoint;
    private boolean isMovingOut;

    public TwoInOneOut(Intake intake) {
        this.intake = intake;
            addRequirements(intake);

        distanceToMoveOut = new LoggedNetworkNumber("TwoInOneOut/distanceOut", 0.04);
        distanceToMoveIn = new LoggedNetworkNumber("TwoInOneOut/distanceToMoveIn", 0.08);
        powerMoveOut = new LoggedNetworkNumber("TwoInOneOut/powerMoveOut", 0.4);
        powerToMoveIn = new LoggedNetworkNumber("TwoInOneOut/powerToMoveIn", 0.4);
        positionTolerance = new LoggedNetworkNumber("TwoInOneOut/positionTolerance", 0.02);
        setpoint = 0;
        
        isMovingOut = true;// start after the intake has been moved out
    }

    public static TwoInOneOut factory(Intake intake){
        return new TwoInOneOut(intake);
    }

    @Override
    public void initialize() {
        isMovingOut = true;
        setpoint = intake.getIntakePosition();
    }

    @Override
    public boolean isFinished(){
        return intake.positionAtSetPoint() && Math.abs(intake.getIntakePosition() - IntakeConstants.CLOSE_POSITION) 
            <= IntakeConstants.POSITION_TOLERANCE;
    }

    public void execute(){
        if (shouldStop() && isMovingOut){
            isMovingOut = false;
            setpoint -= distanceToMoveIn.getAsDouble();
            intake.setPositionMotorPercent(-powerToMoveIn.getAsDouble());
            
        } else if (shouldStop()){
            isMovingOut = true;
            setpoint += distanceToMoveOut.getAsDouble();
            intake.setPositionMotorPercent(powerMoveOut.getAsDouble());
        }
        Logger.recordOutput("TwoInOneOut/isMovingOut", isMovingOut);
        Logger.recordOutput("TwoInOneOut/setpoint", setpoint);
    }

    public void end(boolean interrupted){
        intake.stopIntakeOpeningMotor();
    }

public boolean shouldStop(){
    var shouldStop = isMovingOut ? intake.getIntakePosition() >= setpoint : Math.abs(intake.getIntakePosition() - setpoint) 
            <= positionTolerance.getAsDouble();
    Logger.recordOutput("TwoInOneOut/shouldStop", shouldStop);
    Logger.recordOutput("TwoInOneOut/setpoint", setpoint);
    return shouldStop;
  }
}





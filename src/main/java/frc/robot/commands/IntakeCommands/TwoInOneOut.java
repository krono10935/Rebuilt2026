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

        distanceToMoveOut = new LoggedNetworkNumber("TwoInOneOut/distanceOut", 0.3);
        distanceToMoveIn = new LoggedNetworkNumber("TwoInOneOut/distanceToMoveIn", 0.3);
        powerMoveOut = new LoggedNetworkNumber("TwoInOneOut/powerMoveOut", 0.3);
        powerToMoveIn = new LoggedNetworkNumber("TwoInOneOut/powerToMoveIn", 0.3);
        positionTolerance = new LoggedNetworkNumber("TwoInOneOut/positionTolerance", 0.01);
        setpoint = 0;
        
        isMovingOut = true;// start after the intake has been moved out
    }

    @Override
    public void initialize() {
        isMovingOut = true;
        setpoint = intake.getIntakePosition();
                System.out.println("init two \n");

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

public boolean shouldStop(){
    var shouldStop = isMovingOut ? intake.getIntakePosition() >= setpoint : Math.abs(intake.getIntakePosition() - setpoint) 
            <= positionTolerance.getAsDouble();
    Logger.recordOutput("ShakeBangBang/shouldStop", shouldStop);
    Logger.recordOutput("ShakeBangBang/setpoint", setpoint);
    return shouldStop;
  }

    public static Command factory(Intake intake){
        return new OpenPositionCommand(intake).andThen(new TwoInOneOut(intake));
    }
}





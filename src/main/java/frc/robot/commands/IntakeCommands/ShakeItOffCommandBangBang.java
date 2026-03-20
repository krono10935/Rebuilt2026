// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeItOffCommandBangBang extends Command {
  /** Creates a new ShakeItOffCommandBangBang. */
  private  final Intake intake;

  private final LoggedNetworkNumber openPos;

  private final LoggedNetworkNumber tolerance;

  private final LoggedNetworkNumber openingDutyCycle;

  private final LoggedNetworkNumber closingDutyCycle;

  private final LoggedNetworkNumber closePos;

  private final LoggedNetworkNumber firstClosePos;

  private final LoggedNetworkNumber openLessMultiplier;

  private final Timer beginTimer;

  private double setpoint;

  private int cycles;

  private LoggedNetworkNumber intakeSpeed;
  @AutoLogOutput(key =  "ShakeBangBang/shouldOpen")
  private boolean shouldOpen = false;

  @AutoLogOutput(key = "ShakeBangBang/hasOpened")
  private boolean hasOpened = false;
  public ShakeItOffCommandBangBang(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);


    tolerance = new LoggedNetworkNumber("ShakeBangBang/tolerance", 0.05);
    openingDutyCycle = new LoggedNetworkNumber("ShakeBangBang/openDutyCycle", 0.9);
    closingDutyCycle = new LoggedNetworkNumber("ShakeBangBang/closingDutyCycle", 0.9);
    openPos = new LoggedNetworkNumber("ShakeBangBang/openPos", 0.15);
    firstClosePos = new LoggedNetworkNumber("ShakeBangBang/firstClosePos", 0.075);

    closePos = new LoggedNetworkNumber("ShakeBangBang/closePos", 0.00);
    openLessMultiplier = new LoggedNetworkNumber("ShakeBangBang/openLessMultiplier", 0.7);
    beginTimer = new Timer();
    intakeSpeed = new LoggedNetworkNumber("ShakeBangBang/intakeDutyCycle", 0.5);
    cycles = 0;



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    beginTimer.reset();
    beginTimer.start();

    hasOpened = false;
    shouldOpen = false;

    intake.setPercent(intakeSpeed.getAsDouble());

    setpoint = intake.getIntakePosition();

    cycles = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(beginTimer.get()> 0.5){
      if(shouldStop()){

        if (!shouldOpen && hasOpened){
          cycles++;
        } else if (!shouldOpen) {
          hasOpened = true;
        }

        shouldOpen = !shouldOpen;
        intake.setPositionMotorPercent(shouldOpen ? openingDutyCycle.getAsDouble() *
         Math.pow(Math.sqrt(openLessMultiplier.getAsDouble()), cycles) : 
         -closingDutyCycle.getAsDouble());

        if (hasOpened){
        setpoint = shouldOpen ? openPos.getAsDouble() * Math.pow(openLessMultiplier.getAsDouble(), cycles) :
          closePos.getAsDouble();
        } else {
           setpoint = shouldOpen ? openPos.getAsDouble() * Math.pow(openLessMultiplier.getAsDouble(), cycles) :
          firstClosePos.getAsDouble();
        }
      }
    }
  }

  public boolean shouldStop(){
    var shouldStop = shouldOpen ? intake.getIntakePosition() >= setpoint : Math.abs(intake.getIntakePosition() - setpoint) 
            <= tolerance.getAsDouble();
    Logger.recordOutput("ShakeBangBang/shouldStop", shouldStop);
    Logger.recordOutput("ShakeBangBang/setpoint", setpoint);
    return shouldStop;
  }

  public void end(boolean interrupted){
    intake.stopIntakeOpeningMotor();
    intake.stopIntakeMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeItOffCommandBangBang extends Command {
  /** Creates a new ShakeItOffCommandBangBang. */
  private  final Intake intake;

  private final LoggedNetworkNumber openPos;

  private final LoggedNetworkNumber tolerance;

  private final LoggedNetworkNumber openingDutyCycle;

  private final LoggedNetworkNumber closePos;

  private final LoggedNetworkNumber openLessMultiplier;

  private final Timer timer;

  private final Timer beginTimer;

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
    openingDutyCycle = new LoggedNetworkNumber("ShakeBangBang/openDutyCycle", 0.1);
    openPos = new LoggedNetworkNumber("ShakeBangBang/openPos", 0.25);
    closePos = new LoggedNetworkNumber("ShakeBangBang/closePos", 0.00);
    openLessMultiplier = new LoggedNetworkNumber("ShakeBangBang/openLessMultiplier", 0.75);
    timer = new Timer();
    beginTimer = new Timer();
    intakeSpeed = new LoggedNetworkNumber("ShakeBangBang/intakeDutyCycle", 0.5);
    cycles = 0;



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    timer.reset();
    
    timer.start();

    beginTimer.reset();
    beginTimer.start();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(beginTimer.get()> 0.5){
    if(Math.abs(intake.getIntakePosition() - 
      (shouldOpen ? openPos.getAsDouble() * Math.pow(openLessMultiplier.getAsDouble(), cycles) : 
      closePos.getAsDouble())) < tolerance.getAsDouble()){

      if (!shouldOpen && hasOpened){
        cycles++;
      }

      shouldOpen = ! shouldOpen;
      intake.setPositionMotorPercent(openingDutyCycle.getAsDouble());
      timer.reset();
    }
  }
  }
}

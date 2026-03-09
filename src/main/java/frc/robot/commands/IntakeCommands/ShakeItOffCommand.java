// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.opencv.core.Mat;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeItOffCommand extends Command {
  /** Creates a new ShakeItOffCommand. */
  private  final Intake intake;

  private final LoggedNetworkNumber openPos;

  private final LoggedNetworkNumber tolerance;

  private final LoggedNetworkNumber closePos; 

  private final Timer timer;

  private final LoggedNetworkNumber timeTochange;

  private final Timer beginTimer;

  private LoggedNetworkNumber intakeSpeed;
  @AutoLogOutput(key =  "Shake/shouldOpen")
  private boolean shouldOpen = false;

  @AutoLogOutput(key = "Shake/hasOpened")
  private boolean hasOpened = false;
  public ShakeItOffCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);


    tolerance = new LoggedNetworkNumber("Shake/tolerance", 0.0025);
    openPos = new LoggedNetworkNumber("Shake/openPos", 0.25);
    closePos = new LoggedNetworkNumber("Shake/closePos", 0.00);
    timeTochange = new LoggedNetworkNumber("Shake/time", 1);
    timer = new Timer();
    beginTimer = new Timer();
    intakeSpeed = new LoggedNetworkNumber("Shake/intakeDutyCycle", 0.5);



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
    if(timer.get()>= timeTochange.getAsDouble()){
      shouldOpen = ! shouldOpen;
      intake.setPosition(shouldOpen? openPos.getAsDouble() :closePos.getAsDouble());
      timer.reset();
    }
  }
  }
}

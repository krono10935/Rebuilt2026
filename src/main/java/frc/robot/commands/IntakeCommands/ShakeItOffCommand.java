// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.opencv.core.Mat;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeItOffCommand extends Command {
  /** Creates a new ShakeItOffCommand. */
  private  final Intake intake;

  private final LoggedNetworkNumber openPos;

  private final LoggedNetworkNumber tolerance;

  private final LoggedNetworkNumber closePos; 

  @AutoLogOutput(key =  "Shake/shouldOpen")
  private boolean shouldOpen = false;

  @AutoLogOutput(key = "Shake/hasOpened")
  private boolean hasOpened = false;
  public ShakeItOffCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);


    tolerance = new LoggedNetworkNumber("Shake/tolerance", 0.0025);
    openPos = new LoggedNetworkNumber("Shake/openPos", 0.15);
    closePos = new LoggedNetworkNumber("Shake/closePos", 0.05);



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPosition(openPos.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hasOpened && intake.positionAtSetPoint()) {
      shouldOpen = !shouldOpen;

      intake.setPosition(shouldOpen? openPos.getAsDouble() :closePos.getAsDouble());
      

    }
    else if(!hasOpened && intake.positionAtSetPoint()){
      hasOpened = true;
    }
  }
}

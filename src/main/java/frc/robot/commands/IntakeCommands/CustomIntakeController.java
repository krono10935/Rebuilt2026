// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.CustomControllerConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CustomIntakeController extends Command {
  /** Creates a new CustomIntakeController. */
  private Intake intake;
  private CommandGenericHID intakeController;

  public CustomIntakeController(Intake intake, CommandGenericHID intakeController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeController = intakeController;

    addRequirements(intake);
  }


  private double scale(double axis){
    return IntakeConstants.OPEN_POSITION * axis;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double openAxis = intakeController.getRawAxis(CustomControllerConstants.IntakeControllerConstants.SLIDER_AXIS);
    double spinAxis = intakeController.getRawAxis(CustomControllerConstants.IntakeControllerConstants.FLAP_AXIS);

    double requestPos = scale(-openAxis);

    switch((int)Math.signum(spinAxis)){
      case -1 -> intake.setPercent(-0.9);
      case 1 -> intake.setPercent(0.9);
      case 0 -> intake.stopIntakeMotor();
      default -> intake.stopIntakeMotor();
    }
    
    intake.setPosition(requestPos);
  }
}

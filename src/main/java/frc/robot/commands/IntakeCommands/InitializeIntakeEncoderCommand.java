// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InitializeIntakeEncoderCommand extends Command {
  /** Creates a new InitializeIntakeEncoderCommand. */
  private final Intake intake;
  public InitializeIntakeEncoderCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPositionMotorPercentOutput(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.resetPositionMotorEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getLimitSwitch();
  }
}

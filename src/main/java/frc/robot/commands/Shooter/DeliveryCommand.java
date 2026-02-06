// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliveryCommand extends Command {
  /** Creates a new OongaBoongaShoot. */
  private final Shooter shooter;
  public DeliveryCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinUp(ShooterConstants.DELIVERY_VELOCITY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.keepVelocity(ShooterConstants.DELIVERY_VELOCITY);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isShooterAtGoal();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUp extends Command {
  /** Creates a new SpinUpAndKeepVelocity. */
  private final Shooter shooter;
  private final Drivetrain drivetrain;
  public SpinUp(Shooter shooter, Drivetrain drivetrain) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinUp(
      ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(), drivetrain.getChassisSpeeds())
      .flywheelSpeed());
  }

  @Override
  public boolean isFinished(){
    return shooter.isShooterAtGoal();
  }
}

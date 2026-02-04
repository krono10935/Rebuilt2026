// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShootAndAim extends Command {
  /** Creates a new AutoShootAndAim. */
  private Drivetrain drivetrain;
  private Shooter shooter;
  public AutoShootAndAim(Shooter shooter, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    addRequirements(shooter);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds(), 
     ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getGyroAngle()));

    shooter.spinUp(params.flywheelSpeed());
    shooter.setHoodAngle(params.hoodAngle());
  }

}

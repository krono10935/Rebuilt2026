// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Shooter.ShotCalculator.ValidityState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private final Shooter shooter;

  private final BooleanSupplier thetaAtSetpoint;

  private final Drivetrain drivetrain;
  

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param robotPoseSupplier robot pose supplier
   * @param shouldShootFunction function to translate from pose2d to whether or not to shoot 
   */
  private ShootCommand(Shooter shooter, Drivetrain drivetrain, BooleanSupplier thetaAtSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.thetaAtSetpoint = thetaAtSetpoint;
    this.drivetrain = drivetrain;

    addRequirements(shooter);
  }

  public static Command shootCommandFactory(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller){
    DriveAndHomeCommand driveCommand = new DriveAndHomeCommand(drivetrain, controller);
    ShootCommand shootCommand = new ShootCommand(shooter, drivetrain, driveCommand::atTargetAngle);

    return driveCommand.alongWith(shootCommand);
  }

  @Override
  public void execute() {
    ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds());

    shooter.keepVelocity(params.flywheelSpeed());
    shooter.setHoodAngle(params.hoodAngle());


        // is the robot is in the shooting zone 
    boolean shouldShoot =
      params.validityState() == ValidityState.VALID &&
      thetaAtSetpoint.getAsBoolean() &&
      shooter.readyToShoot();

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
      shooter.toggleKicker(true);
    }

    // otherwise open the kicker and start letting the shooter shoot
    else{
      shooter.toggleKicker(false);
    }
    
  }
}

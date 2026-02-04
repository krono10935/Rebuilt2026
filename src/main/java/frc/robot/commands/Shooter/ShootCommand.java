// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Shooter.ShotCalculator.ValidityState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private final Shooter shooter;

  private final Indexer indexer;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Function<Pose2d, Boolean> shouldShootFunction;

  private final Drivetrain drivetrain;
  

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param robotPoseSupplier robot pose supplier
   * @param shouldShootFunction function to translate from pose2d to whether or not to shoot 
   */
  public ShootCommand(Shooter shooter,Indexer indexer, Supplier<Pose2d> robotPoseSupplier, Function<Pose2d, Boolean> shouldShootFunction) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.indexer = indexer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.shouldShootFunction = shouldShootFunction;
    this.drivetrain = drivetrain;

    addRequirements(shooter, indexer);

  }

  @Override
  public void execute() {
    shooter.updateShootingParameters(drivetrain);
    ShootingParameters params = shooter.getShootParameters();

    shooter.setHoodAngle(Rotation2d.fromDegrees(params.hoodAngle()));
    
    // is the robot is in the shooting zone 
    boolean shouldShoot = shouldShootFunction.apply(robotPoseSupplier.get()) && params.validityState() == ValidityState.VALID;


    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
        indexer.turnOn();
        shooter.toggleKicker(true);

      shooter.toggleKicker(false);

      shooter.spinUp(ShooterConstants.BASE_SPINUP_SPEED);


    }

    // otherwise if the shooter isn't ready to shoot, close to kicker and get it ready to shoot
    else if(!shooter.isShooterAtGoal()){
      
      shooter.toggleKicker(false);

      shooter.spinUp(params.flywheelSpeed());

    }

    // otherwise open the kicker and start letting the shooter shoot
    else{
        indexer.turnOff();
        shooter.toggleKicker(false);
    }

      shooter.keepVelocity();

      shooter.toggleKicker(true);

  @Override
  public void end(boolean interrupted) {
      indexer.turnOff();
      shooter.toggleKicker(false);
      shooter.stopFlyWheel();
  }

}

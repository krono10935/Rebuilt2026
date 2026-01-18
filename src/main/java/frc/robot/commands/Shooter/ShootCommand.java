// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private final Shooter shooter;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Function<Pose2d, Boolean> shouldShootFunction;
  

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param robotPoseSupplier robot pose supplier
   * @param shouldShootFunction function to translate from pose2d to whether or not to shoot 
   */
  public ShootCommand(Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Function<Pose2d, Boolean> shouldShootFunction) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    this.shouldShootFunction = shouldShootFunction;

    addRequirements(shooter);

  }

  @Override
  public void execute() {
    
    // is the robot is in the shooting zone 
    boolean shouldShoot = shouldShootFunction.apply(robotPoseSupplier.get());

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (!shouldShoot){

      shooter.toggleKicker(false);

      shooter.spinUp(100);


    }

    // otherwise if the shooter isn't ready to shoot, close to kicker and get it ready to shoot
    else if(!shooter.isShooterAtSetpoint()){
      
      shooter.toggleKicker(false);

      shooter.shoot(0);

    }

    // otherwise open the kicker and start letting the shooter shoot
    else{

      shooter.toggleKicker(true);

    }
    
  }
}

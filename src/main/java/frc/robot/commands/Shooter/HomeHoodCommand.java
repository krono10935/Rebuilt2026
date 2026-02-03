// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;

@Deprecated
public class HomeHoodCommand extends Command {

  private final Shooter shooter;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Function<Pose2d, Rotation2d> poseToHoodAngle;

  private final Drivetrain drivetrain;
  
  /**
   * 
   * @param shooter the shooter
   * @param robotPoseSupplier supplier for the pose of the robot
   * @param poseToHoodAngle a function to convert the pose to the angle for hood
   */
  public HomeHoodCommand(Shooter shooter,Drivetrain drivetrain, Supplier<Pose2d> robotPoseSupplier, Function<Pose2d, Rotation2d> poseToHoodAngle) {

    this.shooter = shooter;

    this.robotPoseSupplier = robotPoseSupplier;

    this.poseToHoodAngle = poseToHoodAngle;

    this.drivetrain = drivetrain;

    addRequirements(shooter);
  }
  
  /**
   * set the hood to the correct angle
   */
  @Override
  public void execute() {
    shooter.updateShootingParameters(drivetrain);
    shooter.setHoodAngle(shooter.getShootParameters().hoodAngle());
    //shooter.setHoodAngle(poseToHoodAngle.apply(robotPoseSupplier.get()));
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;


public class DriveCommand extends Command {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;

  private static double MAX_LINEAR_SPEED ;
  private static double MIN_LINEAR_SPEED ;
  private static double MAX_ANGULAR_SPEED;
  private static double MIN_ANGULAR_SPEED;
  

  private static final double DEADBAND = 0.1;

  public DriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);

    MAX_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.maxLinearSpeed();
    MIN_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.minLinearSpeed();
    MAX_ANGULAR_SPEED = drivetrain.getConstants().MAX_ANGULAR_SPEED;
    MIN_ANGULAR_SPEED = drivetrain.getConstants().MIN_ANGULAR_SPEED;
  }

  private Rotation2d angleFieldRelative(){
      return ChassisConstants.shouldFlipPath()?
              drivetrain.getGyroAngle():drivetrain.getGyroAngle().rotateBy(Rotation2d.k180deg) ;
  }


  @Override
  public void execute() {
    double speed = lerp(1 - controller.getRightTriggerAxis());
    double angularSpeed = angularLerp(1 - controller.getRightTriggerAxis());

    double xSpeed = deadband(-controller.getLeftX()) * speed;
    double ySpeed = deadband(controller.getLeftY()) * speed;
    double thetaSpeed = deadband(-controller.getRightX()) * angularSpeed;

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, angleFieldRelative()));
  }

  /**
   * Calculate the linear lerp of a {@code value}
   * @param value
   * @return the linear lerp of a {@code value}
   */
  private static double lerp(double value){
    return MIN_LINEAR_SPEED + (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * value;
  }

  /**
   * Calculate the angular lerp of a {@code value}
   * @param value
   * @return the angular lerp of a {@code value}
   */
  private static double angularLerp(double value){
    return MIN_ANGULAR_SPEED + (MAX_ANGULAR_SPEED - MIN_ANGULAR_SPEED) * value;
  }

  /**
   * Calculate if the value is passed the deadband value
   * @param value
   * @return 0 if the absolute {@code value} is less than deadband, otherwise {@code value}
   */
  private static double deadband(double value){
    if (Math.abs(value) < DEADBAND){
      return 0;
    }

    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DriveAndHomeToHubCommand;
import frc.robot.commands.IntakeCommands.ShakeItOffCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private static boolean overrideObjectDetection = true;

  private final Shooter shooter;

  private final CommandXboxController controller;

  private final LoggedNetworkNumber speedMPS;

  private final LoggedNetworkNumber hoodAngleDegrees;

  private final Timer hoodSetpointTimer;
  private final Alert hoodFailedToSetpoint;

  private final Timer flyWheelSetpointTimer;
  private final Alert flyWheelFailedToSetpoint;

  private final Timer kickerStuckTimer;
  private final Alert kickerStuck;

  private final Timer indexerStuckTimer;
  private final Alert indexerStuck;

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param drivetrain drivetrain
   */
  public BasicShootCommand(Shooter shooter, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;

    this.controller = controller;

    speedMPS = new LoggedNetworkNumber("speedMPS", 17);

    hoodAngleDegrees = new LoggedNetworkNumber("hoodAngleDegrees", 22);

    hoodSetpointTimer = new Timer();
    hoodFailedToSetpoint = new Alert("Hood failed to reach setpoint", AlertType.kError);

    flyWheelSetpointTimer = new Timer();
    flyWheelFailedToSetpoint = new Alert("Flywheel failed to reach setpoint", AlertType.kError);

    kickerStuckTimer = new Timer();
    kickerStuck = new Alert("The kicker is stuck!", AlertType.kError);

    indexerStuckTimer = new Timer();
    indexerStuck = new Alert("The indexer is stuck!", AlertType.kError);

    addRequirements(shooter, shooter.getIndexer());
  }


  private boolean hasReachedTargetVelocity = false;
  private double lastTargetVelocity = 0;

  @Override
  public void execute() {

     double targetFlywheelSpeed = speedMPS.getAsDouble();
     Rotation2d targetHoodAngle = Rotation2d.fromDegrees(hoodAngleDegrees.getAsDouble());

    if(Math.abs(targetFlywheelSpeed - lastTargetVelocity) > 0.3){
      lastTargetVelocity = targetFlywheelSpeed;
      hasReachedTargetVelocity = false;
    }

    shooter.keepVelocity(targetFlywheelSpeed);
    shooter.setHoodAngle(targetHoodAngle);


    if(!hasReachedTargetVelocity && shooter.isShooterAtGoal()){
      hasReachedTargetVelocity = true;
    }


    Logger.recordOutput("TestShootCommand/hood", shooter.isHoodAtSetpoint());
    Logger.recordOutput("TestShootCommand/shooter", hasReachedTargetVelocity);
//    Logger.recordOutput("ShootCommand/thetaAtSetpoint", thetaAtSetpoint);

        // is the robot is in the shooting zone
    boolean shouldShoot =
      hasReachedTargetVelocity && shooter.isHoodAtSetpoint() &&
      (ObjectDetection.getInstance().hasBalls() || !Constants.USE_OBJECT_DETECTION
       || overrideObjectDetection);

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
      handleHoodErrors();
      handleFlyWheelErrors();
      handleIndexerErrors();
      handleKickerErrors();

      shooter.toggleKicker(true);
      if (controller.a().getAsBoolean()) {
        shooter.getIndexer().reverse();
      } else {
        shooter.getIndexer().turnOn();
      }
    }

    // otherwise open the kicker and start letting the shooter shoot
    else{
      shooter.toggleKicker(false);
      shooter.getIndexer().turnOff();
    }
  }

  private void handleHoodErrors(){
    if (!hoodSetpointTimer.isRunning()){
      hoodSetpointTimer.start();
    }

    if (shooter.isHoodAtSetpoint()){
        hoodSetpointTimer.reset();
        hoodFailedToSetpoint.set(false);
      } else if (hoodSetpointTimer.get() > ShootRealConstants.HOOD_SETPOINT_ARRIVAL_TIME){
        hoodFailedToSetpoint.set(true);
      }
  }

  private void handleFlyWheelErrors(){
    if (!flyWheelSetpointTimer.isRunning()){
      flyWheelSetpointTimer.start();
    }

    if (shooter.isShooterAtGoal()){
        flyWheelSetpointTimer.reset();
        flyWheelFailedToSetpoint.set(false);

      } else if (flyWheelSetpointTimer.get() > ShootRealConstants.FLYWHEEL_TIME_TO_REACH_GOAL){
        flyWheelFailedToSetpoint.set(true);
      }
  }

  private void handleKickerErrors(){
    if (!kickerStuckTimer.isRunning()){
      kickerStuckTimer.start();
    }

    if (shooter.isKickerActive()){
        kickerStuckTimer.reset();
        kickerStuck.set(false);
      } else if (kickerStuckTimer.get() > ShootRealConstants.TIME_TO_NOT_BE_DEADBAND){
        kickerStuck.set(true);
      }
  }

  private void handleIndexerErrors(){
    if (!indexerStuckTimer.isRunning()){
      indexerStuckTimer.start();
    }

    if (!shooter.getIndexer().isStuck()){
        indexerStuckTimer.reset();
        indexerStuck.set(false);

      } else if (indexerStuckTimer.get() > ShootRealConstants.TIME_TO_NOT_BE_DEADBAND){
        indexerStuck.set(true);
      }
  }

  @Override
  public void end(boolean interrupted){
    shooter.stopFlyWheel();
    shooter.toggleKicker(false);
    shooter.getIndexer().turnOff();
  }


  public static Command shootCommandFactory(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller, Intake intake, Vision vision){
    DriveAndHomeToHubCommand driveCommand = new DriveAndHomeToHubCommand(drivetrain, controller);
    Command shootCommand = (
        new ShootCommand(shooter, drivetrain, vision, () ->false )  
        .alongWith((new ShakeItOffCommand(intake)))
      ).beforeStarting(new SpinUp(shooter, drivetrain));

    return driveCommand.alongWith(shootCommand).withName("Full Shoot");
  }

  public static void setOverrideObjectDetection(boolean mode){
    overrideObjectDetection = mode;
  }
}

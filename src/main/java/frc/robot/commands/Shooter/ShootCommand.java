// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.lang.invoke.ConstantBootstraps;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Shooter.ShotCalculator.ValidityState;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private final Shooter shooter;

  private final Vision vision;

  private final Drivetrain drivetrain;

  private final Intake intake;

  private final Rotation2d INTAKE_CLOSING_VELOCITY = Rotation2d.fromDegrees(10);

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param drivetrain drivetrain
   */
  public ShootCommand(Shooter shooter, Drivetrain drivetrain, Intake intake, Vision vision ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.vision = vision;

      addRequirements(shooter);
  }

  public static Command shootCommandFactory(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller, Intake intake, Vision vision){
    DriveAndHomeCommand driveCommand = new DriveAndHomeCommand(drivetrain, controller);
    ShootCommand shootCommand = new ShootCommand(shooter, drivetrain, intake, vision);

    return driveCommand.alongWith(shootCommand);
  }

  @Override
  public void initialize(){
    vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);
  }

  private boolean hasReachedTargetVelocity = false;
  private double lastTargetVelocity = 0;

  @Override
  public void execute() {
    ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds());

    if(Math.abs(params.flywheelSpeed() - lastTargetVelocity) > 0.3){
      lastTargetVelocity = params.flywheelSpeed();
      hasReachedTargetVelocity = false;
    }

    shooter.keepVelocity(params.flywheelSpeed());
    shooter.setHoodAngle(params.hoodAngle());

    boolean thetaAtSetpoint = Math.abs(drivetrain.getEstimatedPosition().getRotation().getRadians() -
            params.robotAngle().getRadians()) <= DriveAndHomeCommand.robotAngleTolerance.getRadians();

    if(!hasReachedTargetVelocity && shooter.isShooterAtGoal()){
      hasReachedTargetVelocity = true;
    }

    Logger.recordOutput("ShootCommand/thetaAtSetpoint", thetaAtSetpoint);
    Logger.recordOutput("ShootCommand/validity state", params.validityState());
    Logger.recordOutput("ShootCommand/hood", shooter.isHoodAtSetpoint());
    Logger.recordOutput("ShootCommand/shooter", hasReachedTargetVelocity);
//    Logger.recordOutput("ShootCommand/thetaAtSetpoint", thetaAtSetpoint);

        // is the robot is in the shooting zone
    boolean shouldShoot =
      params.validityState() == ValidityState.VALID &&
      thetaAtSetpoint &&
      hasReachedTargetVelocity && shooter.isHoodAtSetpoint() &&
      (ObjectDetection.getInstance().hasBalls() || !Constants.USE_OBJECT_DETECTION);

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
      shooter.toggleKicker(true);
      shooter.getIndexer().turnOn();
      //intake.setPositionMotorVelocity(INTAKE_CLOSING_VELOCITY.times(intake.getIntakePosition() / IntakeConstants.OPEN_POSITION));
    }

    // otherwise open the kicker and start letting the shooter shoot
    else{
      shooter.toggleKicker(false);
      shooter.getIndexer().turnOff();
      //intake.setPositionMotorVelocity(Rotation2d.kZero);

    }
  }

  @Override
  public void end(boolean interrupted){
    shooter.stopFlyWheel();
    shooter.toggleKicker(false);
    shooter.getIndexer().turnOff();
  }
}

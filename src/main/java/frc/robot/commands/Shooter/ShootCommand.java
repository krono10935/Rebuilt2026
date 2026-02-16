// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Shooter.ShotCalculator.ValidityState;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

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
    vision.setCamAsPriority(CamerasConstants.FRONT_CAMERA);
  }

  @Override
  public void execute() {
    ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds());

    shooter.keepVelocity(params.flywheelSpeed());
    shooter.setHoodAngle(params.hoodAngle());

    boolean thetaAtSetpoint = Math.abs(drivetrain.getEstimatedPosition().getRotation().getRadians() -
            params.robotAngle().getRadians()) <= DriveAndHomeCommand.robotAngleTolerance.getRadians();

        // is the robot is in the shooting zone
    boolean shouldShoot =
      params.validityState() == ValidityState.VALID &&
      thetaAtSetpoint &&
      shooter.readyToShoot();

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
      shooter.toggleKicker(true);
      shooter.getIndexer().turnOn();
      intake.setPositionMotorVelocity(INTAKE_CLOSING_VELOCITY);
    }

    // otherwise open the kicker and start letting the shooter shoot
    else{
      shooter.toggleKicker(false);
      shooter.getIndexer().turnOn();
      intake.setPositionMotorVelocity(Rotation2d.kZero);

    }

    
  }
}

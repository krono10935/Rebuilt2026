// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.lang.Character.Subset;
import java.lang.invoke.ConstantBootstraps;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DriveAndHomeToHubCommand;
import frc.robot.Sequences;
import frc.robot.commands.Drivetrain.DriveAndHomeCommand;
import frc.robot.commands.IntakeCommands.CloseSlowAndThenFast;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.IntakeCommands.ShakeItOffCommand;
import frc.robot.commands.IntakeCommands.ShakeItOffCommandBangBang;
import frc.robot.commands.IntakeCommands.SlowlyClose;
import frc.robot.commands.IntakeCommands.TwoInOneOut;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Shooter.ShotCalculator.ValidityState;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private static Rotation2d hoodOffset = Rotation2d.kZero;

  private static double shooterSpeedOffset = 0;

  private static boolean overrideObjectDetection = true;

  private final Shooter shooter;

  private final Vision vision;

  private final Drivetrain drivetrain;

  private final Timer hoodSetpointTimer;
  private final Alert hoodFailedToSetpoint;

  private final Timer flyWheelSetpointTimer;
  private final Alert flyWheelFailedToSetpoint;

  private final Timer kickerStuckTimer;
  private final Alert kickerStuck;

  private final Timer indexerStuckTimer;
  private final Alert indexerStuck;

  private final Rotation2d INTAKE_CLOSING_VELOCITY = Rotation2d.fromDegrees(10);

  private final BooleanSupplier reverseIndexer;

  /**
   * 
   * @param shooter subsystem to activate the shoot command on
   * @param drivetrain drivetrain
   */
  public ShootCommand(Shooter shooter, Drivetrain drivetrain, Vision vision, BooleanSupplier reverseIndexer ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.reverseIndexer  = reverseIndexer;

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

  @Override
  public void initialize(){
    vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);

    hoodOffset = Rotation2d.kZero;
    shooterSpeedOffset = 0;
  }


  private boolean hasReachedTargetVelocity = false;
  private double lastTargetVelocity = 0;

  @Override
  public void execute() {
    ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
     drivetrain.getChassisSpeeds());

     double targetFlywheelSpeed = params.flywheelSpeed() + shooterSpeedOffset;
     Rotation2d targetHoodAngle = params.hoodAngle().plus(hoodOffset);

    if(Math.abs(targetFlywheelSpeed - lastTargetVelocity) > 0.3){
      lastTargetVelocity = targetFlywheelSpeed;
      hasReachedTargetVelocity = false;
    }

    shooter.keepVelocity(targetFlywheelSpeed);
    shooter.setHoodAngle(targetHoodAngle);

    boolean thetaAtSetpoint = Math.abs(drivetrain.getEstimatedPosition().getRotation().minus(params.robotAngle()).getRadians()) <= DriveAndHomeToHubCommand.robotAngleTolerance.getRadians();
    boolean thetaAtSetpoint = Math.abs(drivetrain.getEstimatedPosition().getRotation().minus(params.robotAngle()).getRadians()) <= DriveAndHomeCommand.robotAngleTolerance.getRadians();

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
      (ObjectDetection.getInstance().hasBalls() || !Constants.USE_OBJECT_DETECTION
       || overrideObjectDetection);

    // robot it isn't in shooting zone, go to spin up mode and turn off kicker
    if (shouldShoot){
      handleHoodErrors();
      handleFlyWheelErrors();
      handleIndexerErrors();
      handleKickerErrors();

      shooter.toggleKicker(true);
      shooter.getIndexer().setSpeed(reverseIndexer.getAsBoolean()? -IndexerConstants.SPINNING_TARGET_VELOCITY : 
      IndexerConstants.SPINNING_TARGET_VELOCITY);
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

  public static void AddToHoodOffset(Rotation2d offset){
    hoodOffset = hoodOffset.plus(offset);
  }

  public static void AddToFlywheelOffset(double offset){
    shooterSpeedOffset += offset;
  }

  public static Command shootCommandFactory(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller, Intake intake, Vision vision){
    DriveAndHomeToHubCommand driveCommand = new DriveAndHomeToHubCommand(drivetrain, controller);
    Command shootCommand = (
        new ShootCommand(shooter, drivetrain, vision, () -> controller.leftBumper().getAsBoolean())  
        .alongWith(new ShakeItOffCommandBangBang(intake))
      ).beforeStarting(new SpinUp(shooter, drivetrain));

    return driveCommand.alongWith(shootCommand).withName("Full Shoot");
  }


  public static Command shootCommandFactoryStaticIntake(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller, Intake intake, Vision vision){
    DriveAndHomeCommand driveCommand = new DriveAndHomeCommand(drivetrain, controller);
    Command shootCommand = (
        new ShootCommand(shooter, drivetrain, vision, () -> controller.leftBumper().getAsBoolean())
      ).beforeStarting(new SpinUp(shooter, drivetrain)).alongWith(Sequences.intakeOpenStart(intake));

    return driveCommand.alongWith(shootCommand).withName("Full Shoot");
  }

  public static void setOverrideObjectDetection(boolean mode){
    overrideObjectDetection = mode;
  }
}

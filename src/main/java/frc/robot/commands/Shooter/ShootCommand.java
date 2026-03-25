// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveAndHomeToHubCommand;
import frc.robot.commands.IntakeCommands.TwoInOneOut;
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
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
    /** Creates a new ShootCommand. */

    private static boolean overrideObjectDetection = true;

    private final Shooter shooter;

    private final Vision vision;

    private final Drivetrain drivetrain;

    private final Intake intake;

    private final Timer hoodSetpointTimer;
    private final Alert hoodFailedToSetpoint;

    private final Timer flyWheelSetpointTimer;
    private final Alert flyWheelFailedToSetpoint;

    private final Timer kickerStuckTimer;
    private final Alert kickerStuck;

    private final Timer indexerStuckTimer;
    private final Alert indexerStuck;

    private final BooleanSupplier reverseIndexer;

    
    private boolean hasReachedTargetVelocity = false;
    private double lastTargetVelocity = 0;

    private Supplier<IntakeMode> intakeModeSupplier;
    private IntakeMode previousIntakeMode = null;
    private IntakeMode currentIntakeMode = null;

    /**
     * 
     * @param shooter subsystem to activate the shoot command on
     * @param drivetrain drivetrain
     */
    public ShootCommand(Shooter shooter, Drivetrain drivetrain, Vision vision, Intake intake,
             BooleanSupplier reverseIndexer, Supplier<IntakeMode> intakeModeSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.intake = intake;
        this.reverseIndexer  = reverseIndexer;

        hoodSetpointTimer = new Timer();
        hoodFailedToSetpoint = new Alert("Hood failed to reach setpoint", AlertType.kError);

        flyWheelSetpointTimer = new Timer();
        flyWheelFailedToSetpoint = new Alert("Flywheel failed to reach setpoint", AlertType.kError);

        kickerStuckTimer = new Timer();
        kickerStuck = new Alert("The kicker is stuck!", AlertType.kError);

        indexerStuckTimer = new Timer();
        indexerStuck = new Alert("The indexer is stuck!", AlertType.kError);

        this.intakeModeSupplier = intakeModeSupplier;

        addRequirements(shooter, shooter.getIndexer());
    }

    @Override
    public void initialize(){
        vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);

        currentIntakeMode = intakeModeSupplier.get();

    }

    @Override
    public void execute() {
        ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
        drivetrain.getChassisSpeeds());

        double targetFlywheelSpeed = params.flywheelSpeed();
        Rotation2d targetHoodAngle = params.hoodAngle();

        if(Math.abs(targetFlywheelSpeed - lastTargetVelocity) > 0.3){
            lastTargetVelocity = targetFlywheelSpeed;
            hasReachedTargetVelocity = false;
        }

        shooter.keepVelocity(targetFlywheelSpeed);
        shooter.setHoodAngle(targetHoodAngle);

        boolean thetaAtSetpoint = Math.abs(drivetrain.getEstimatedPosition().getRotation().minus(params.robotAngle()).getRadians()) <= DriveAndHomeToHubCommand.robotAngleTolerance.getRadians();

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
            (ObjectDetection.getInstance().hasBalls()
            || overrideObjectDetection);

        // robot it isn't in shooting zone, go to spin up mode and turn off kicker
        if (shouldShoot || RobotContainer.getInstance().overrideShooting){
            handleHoodErrors();
            handleFlyWheelErrors();
            handleIndexerErrors();
            handleKickerErrors();

            if(RobotState.isTeleop()) {            
                IntakeMode.chooseMode(intakeModeSupplier.get());
                IntakeMode.dealWithChosenMode(intake);
            }

            shooter.toggleKicker(true);
            shooter.getIndexer().setSpeed(reverseIndexer.getAsBoolean()? -IndexerConstants.SPINNING_TARGET_VELOCITY : 
            IndexerConstants.SPINNING_TARGET_VELOCITY);
        }

        // otherwise open the kicker and start letting the shooter shoot
        else{
            IntakeMode.resetLastChosen(intake);
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

        if(currentIntakeMode.getCommand(intake).isScheduled()){
            currentIntakeMode.getCommand(intake).cancel();
        }
        previousIntakeMode = null;

        shooter.stopFlyWheel();
        shooter.toggleKicker(false);
        shooter.getIndexer().turnOff();
    }

    public static Command shootCommandFactory(Shooter shooter, Drivetrain drivetrain, CommandXboxController controller,
     Intake intake, Vision vision, BooleanSupplier invertIndexer, Supplier<IntakeMode> intakeModeSupplier){
        DriveAndHomeToHubCommand driveCommand = new DriveAndHomeToHubCommand(drivetrain, controller);
        Command shootCommand = (
            new ShootCommand(shooter, drivetrain, vision, intake, invertIndexer, intakeModeSupplier)
        ).beforeStarting(new SpinUp(shooter, drivetrain));

        return driveCommand.alongWith(shootCommand).withName("Full Shoot");
    }


    public static Command basicShootCommandFactory(Shooter shooter, Intake intake, CommandXboxController controller){
        Command shootCommand = (
            new BasicShootCommand(shooter, controller)  
            .alongWith(new TwoInOneOut(intake))
        ).beforeStarting(new InstantCommand(() -> shooter.spinUp(17)));

        return shootCommand.withName("Basic Shoot");
    }

    public static void setOverrideObjectDetection(boolean mode){
        overrideObjectDetection = mode;
    }
}

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.utils.controllers.ControllerMultiplierType;
import frc.utils.controllers.ExponentialCommandXboxController;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.DriveToPoseConstants;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drivetrain.DriveAndHomeCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.DriveRobotRelative;
import frc.robot.commands.Shooter.BasicShootCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.commands.Shooter.SpinUpForEnterTrench;
import frc.robot.leds.LedManager;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.PPController;
import frc.robot.subsystems.intake.Intake;
import frc.utils.AllianceFlipUtil;
import frc.utils.CheckFreeSpace;
import frc.utils.Elastic;


public class RobotContainer
{
    private static RobotContainer instance;

    public final LedManager ledManager;

    public final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    private final ExponentialCommandXboxController driverController;

    private final CommandGenericHID operatorController;

    public final Drivetrain drivetrain;

    private Trigger AutoShoot;

    private Trigger test;

    private final LoggedDashboardChooser<Command> autoChooser;



    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        shooter = new Shooter();

        ledManager = new LedManager();

        intake = new Intake();

        driverController = new ExponentialCommandXboxController(0, ControllerMultiplierType.SQRT);

        operatorController = new CommandGenericHID(1);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        autoChooser = registerNamedCommand(new DriveAndHomeCommand(drivetrain, driverController));

        ObjectDetection.getInstance();

        DriveToPoseConstants.MAX_LINEAR_SPEED = 4.5;
        DriveToPoseConstants.POSE_TOLERANCE = 0.01;

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        SmartDashboard.putNumber("Robot/Disk Used Space Percent", CheckFreeSpace.checkUsedPercentage()); //TODO: fix
        Logger.recordOutput("Robot/Disk Used Space Percent", CheckFreeSpace.checkUsedPercentage());

        CommandScheduler.getInstance().schedule(ShotCalculator.getInstance().warmUpShotCalculator());
        
        // TODO enable for comp
        // if (ModeFileHandling.isCompMode()){
        //     configureBindings();
        // } else {
        //     configurePitBindings();
        // }
        test();
    }

    /**
     * Test bindings
     */
    private void test(){
        driverController.a().toggleOnTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision));
        driverController.b().onTrue(Sequences.intakeOpenStart(intake));
        driverController.x().toggleOnTrue(Sequences.stopIntakeAndClose(intake));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain,driverController));

        driverController.y().onTrue(IntakeFactory.resetIntake(intake));
        
        //TODO: add intake outtake functionality

        driverController.rightBumper().onTrue(drivetrain.resetGyro());

        //driverController.leftBumper().onTrue(new InstantCommand(() -> shooter.getIndexer().reverse()));

        // Trigger closeEnoughToSpinUp = new Trigger(()
        //         -> drivetrain.getEstimatedPosition().getTranslation().getDistance(
        //         FieldConstants.getClosestTrench(drivetrain.getEstimatedPosition())
        // ) < ShooterConstants.MIN_DISTANCE_FROM_AZ_TO_SPINUP);

        // closeEnoughToSpinUp.and(RobotState::isTeleop).whileTrue(new SpinUpForEnterTrench(shooter,drivetrain).onlyIf(() ->
        //                 shooter.getCurrentCommand() == shooter.getDefaultCommand()));

        BooleanSupplier isHubActive = () -> {
            double time = DriverStation.getMatchTime();

            return Constants.HubTiming.isActive(time) ||
                    Constants.HubTiming.isActive(time - Constants.HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE) ||
                    Constants.HubTiming.isActive(time + Constants.HUB_ACTIVITY_DEABAND_AFTER_ACTIVE);
        };

        AutoShoot = new Trigger(RobotState::isTeleop).
                and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition())
                        && ObjectDetection.getInstance().hasBalls())
                .whileTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision))
                .onFalse(shooter.resetShooterCommand().alongWith(IntakeFactory.resetIntake(intake)));
                
        Logger.recordOutput("alliancePose", FieldConstants.Hub.topCenterPoint);
        Logger.recordOutput("alliancePoseAPplu", AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));


    }

    /**
     * Configure bindings for the Pit.
     */
    private void configurePitBindings() {
        //TODO test these
        LoggedNetworkNumber spinUpSpeedMPS = new LoggedNetworkNumber("spinUpSpeedMPS", 10);
        driverController.a().onTrue(new BasicShootCommand(shooter).beforeStarting(
            new InstantCommand(() -> shooter.spinUp(spinUpSpeedMPS.getAsDouble()))
            .withDeadline(new WaitUntilCommand(shooter::isHoodAtSetpoint))));

        // Disable all subsystems commands
        driverController.b().onTrue(new InstantCommand(() -> {
            drivetrain.getCurrentCommand().cancel();
            drivetrain.stop();
            CommandScheduler.getInstance().schedule(drivetrain.idle());

            shooter.getCurrentCommand().cancel();
            shooter.stopFlyWheel();
            shooter.toggleKicker(false);
            shooter.getIndexer().turnOff();
            CommandScheduler.getInstance().schedule(shooter.idle());

            intake.getCurrentCommand().cancel();
            intake.stopIntakeMotor();
            intake.stopIntakeOpeningMotor();
            CommandScheduler.getInstance().schedule(intake.idle());
        }));

        driverController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        driverController.y().onFalse(new CloseCommand(intake));

        driverController.x().whileTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        driverController.rightBumper().onTrue(drivetrain.resetGyro());
    }

    /**
     * Configure the bindings for the match
     */
    private void configureBindings() {
        //TODO test these
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
        
        driverController.y().toggleOnTrue(Sequences.intakeOpenStart(intake).alongWith(new DriveRobotRelative(drivetrain, driverController)));

        driverController.x().onTrue(
                Sequences.delivery(drivetrain, shooter, driverController,intake));

        driverController.a().onTrue(drivetrain.resetGyro());

        operatorController.povUp().onTrue(new InstantCommand(() -> 
            ShootCommand.AddToHoodOffset(ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK))
        );

        operatorController.povDown().onTrue(new InstantCommand(() -> 
            ShootCommand.AddToHoodOffset(ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK.unaryMinus()))
        );

        operatorController.povRight().onTrue(new InstantCommand(() -> 
            ShootCommand.AddToFlywheelOffset(ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK))
        );

        operatorController.povLeft().onTrue(new InstantCommand(() -> 
            ShootCommand.AddToFlywheelOffset(-ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK))
        );

        operatorController.button(1).onTrue(    
            new InstantCommand(() -> ShootCommand.setOverrideObjectDetection(true))

        ).onFalse(new InstantCommand(() -> ShootCommand.setOverrideObjectDetection(false)));

        
        Trigger closeEnoughToSpinUp = new Trigger(()
            -> drivetrain.getEstimatedPosition().getTranslation().getDistance(
                FieldConstants.getClosestTrench(drivetrain.getEstimatedPosition())
            ) < ShooterConstants.MIN_DISTANCE_FROM_AZ_TO_SPINUP);
        
        closeEnoughToSpinUp.and(RobotState::isTeleop).whileTrue(new SpinUpForEnterTrench(shooter,drivetrain).onlyIf(() ->
        shooter.getCurrentCommand() == shooter.getDefaultCommand()))
            .onFalse(new InstantCommand(()-> shooter.stopFlyWheel(), shooter));

        BooleanSupplier isHubActive = () -> {
            double time = DriverStation.getMatchTime();

            return Constants.HubTiming.isActive(time) ||
                    Constants.HubTiming.isActive(time - Constants.HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE) ||
                    Constants.HubTiming.isActive(time + Constants.HUB_ACTIVITY_DEABAND_AFTER_ACTIVE);
        };

        new Trigger(isHubActive).and(RobotState::isTeleop).
                and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition()))
                .whileTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision));
    }

    /**
     * @return the chosen autonomous command.
     */
    public Command getAutonomousCommand()
    {
        var selectedAuto = autoChooser.get();

        Command autoCommand = Commands.sequence(
            IntakeFactory.resetIntake(intake), selectedAuto, drivetrain.idle());

        CommandScheduler.getInstance().removeComposedCommand(selectedAuto);

        return autoCommand.withName(selectedAuto.getName());
    }

    /**
     * Displays the path the auto {@code command} takes
     * @param command the command runnning in auto
     */
    private void displayChosenAuto(Command command){
        if(RobotState.isEnabled()){
            drivetrain.clearFiledPath();
            return;
        }

        List<PathPlannerPath> auto;

        try{
            auto = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
        }
        catch(IOException | ParseException e){
            Logger.recordOutput("autoDisplay", e.getMessage());
            drivetrain.clearFiledPath();
            return;
        }

        ArrayList<Pose2d> poses = new ArrayList<>();
        for(PathPlannerPath path : auto){
            path = ChassisConstants.shouldFlipPath() ? path : path.flipPath();
            poses.addAll(path.getPathPoses());
        }

        drivetrain.addPathToField(poses);
    }


    /**
     * @param driveAndHomeCommand that pathplanner will use (replaces it with a PPController)
     * @return A LoggedDashboardChooser for the auto commands and gives
     * PathPlanner sequences for our auto commands
     */
    public LoggedDashboardChooser<Command> registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        Command aimRobot = new StartEndCommand(() -> {
            driveAndHomeCommand.resetThetaController();
            PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain)
                .beforeStarting(driveAndHomeCommand::resetThetaController);


        NamedCommands.registerCommand("shootAndAimMoving",
                ( (new ShootCommand(shooter, drivetrain, vision , () -> false)).alongWith(new ShakeItOffCommand(intake))).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                ( (new ShootCommand(shooter, drivetrain, vision, () -> false )).alongWith(new ShakeItOffCommand(intake))).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new RunCommand(() -> shooter.spinUp(17.5), shooter));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                !ObjectDetection.getInstance().hasBalls()).andThen(new WaitCommand(0.3))
                .andThen(Commands.print("no balls")));

         NamedCommands.registerCommand("openIntake",
                 new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
         NamedCommands.registerCommand("closeIntake",
                 new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
        autoChooser.onChange(this::displayChosenAuto);
        return autoChooser;
    }
}
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.leds.LED;
import frc.robot.subsystems.UpdateWigdets.UpdateWidgets;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.AutoLogOutput;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HubTiming;
import frc.robot.commands.Shooter.BasicShootCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.leds.LedManager;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.PPController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.utils.AllianceFlipUtil;


public class RobotContainer {
    private static RobotContainer instance;

    public final LedManager ledManager;

    public final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    private final CommandXboxController driverController;

    private final CommandXboxController operatorController;

    public final Drivetrain drivetrain;

    private LED led;
    
    private final LoggedDashboardChooser<Command> autoChooser;

    public boolean overrideShooting = false;

    @AutoLogOutput(key = "ShootNOW")
    public boolean immediatelyShoot = false;

    public boolean cancelAutomations = false;

    public IntakeMode currentIntakeMode = IntakeMode.FourtyBalls;

    public boolean closeIntakeImmediately = false;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer() {
        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        shooter = new Shooter();

        ledManager = new LedManager();

        led = LED.getInstance();

        intake = new Intake();

        driverController = new CommandXboxController(0);

        operatorController = new CommandXboxController(1);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        autoChooser = registerNamedCommand(new DriveAndHomeToHubCommand(drivetrain, driverController));

        ObjectDetection.getInstance();
        new UpdateWidgets();

        DriveToPoseConstants.MAX_LINEAR_SPEED = 4.5;
        DriveToPoseConstants.POSE_TOLERANCE = 0.01;

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());


        CommandScheduler.getInstance().schedule(ShotCalculator.getInstance().warmUpShotCalculator());

        CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
            if (DriverStation.getAlliance().isPresent())
            led.setDefaultPattern(DriverStation.getAlliance().get() == Alliance.Red);
            led.putDefaultPattern();
            led.setLEDState(true);}).ignoringDisable(true));



        // TODO enable for comp
        // if (ModeFileHandling.isCompMode()){
        //     configureBindings();
        // } else {
        //     configurePitBindings();
        // }

        IntakeMode.initializeLinkedList();

        configureBindings();
        configureOperatorBindings();
//        test();
    }

    /**
     * Test bindings
     */
    private void test() {
        //driverController.a().toggleOnTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision));
        driverController.b().onTrue(Sequences.intakeOpenStart(intake));
        driverController.x().onTrue(Sequences.stopIntakeAndClose(intake));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
        //driverController.a().onTrue(new InstantCommand(shooter.getIndexer()::turnOn));
        SwerveSysID sysid = new SwerveSysID(drivetrain, driverController);

        LoggedNetworkNumber volt = new LoggedNetworkNumber("drivetrain/kstest", 0);

        Command spin = new RunCommand(() -> sysid.spin(volt.getAsDouble()));

        spin.addRequirements(drivetrain);

        driverController.a().onTrue(new DriveAndHomeToIntake(drivetrain, driverController));

        driverController.povUp()
                .toggleOnTrue(ShootCommand.basicShootCommandFactory(shooter, intake, operatorController));

        driverController.leftBumper().whileTrue(new InstantCommand(() -> intake.setRollerDutyCycle(-0.9)))
                .onFalse(new InstantCommand(() -> intake.stopIntakeRoller()));

        driverController.y().onTrue(IntakeFactory.resetIntake(intake));

        //TODO: add intake outtake functionality

        driverController.rightBumper().onTrue(drivetrain.resetGyro());

        operatorController.a().whileTrue(new InstantCommand(() -> shooter.getIndexer().spinBackward(), shooter.getIndexer()))
                .onFalse(new InstantCommand(() -> shooter.getIndexer().turnOff(), shooter.getIndexer()));

        operatorController.b().whileTrue(new InstantCommand(() -> shooter.getIndexer().spinForward(), shooter.getIndexer()))
                .onFalse(new InstantCommand(() -> shooter.getIndexer().turnOff(), shooter.getIndexer()));

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

//        AutoShoot = new Trigger(RobotState::isTeleop).
//                and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition())
//                        && ObjectDetection.getInstance().hasBalls())
//                .whileTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision))
//                .onFalse(shooter.resetShooterCommand().alongWith(IntakeFactory.resetIntake(intake)));

        Logger.recordOutput("alliancePose", FieldConstants.Hub.topCenterPoint);
        Logger.recordOutput("alliancePoseAPplu", AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));


    }

    /**
     * Configure bindings for the Pit.
     */
    private void configurePitBindings() {
        //TODO test these
        LoggedNetworkNumber spinUpSpeedMPS = new LoggedNetworkNumber("spinUpSpeedMPS", 10);
        driverController.a().onTrue(new BasicShootCommand(shooter, operatorController).beforeStarting(
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
            intake.stopIntakeRoller();
            intake.stopPositionMotor();
            CommandScheduler.getInstance().schedule(intake.idle());
        }));

        driverController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        driverController.y().onFalse(new CloseCommand(intake));

        driverController.x().whileTrue(new IntakeCommand(intake).onlyIf(intake::isFullyOpen));

        driverController.rightBumper().onTrue(drivetrain.resetGyro());
    }

    /**
     * Configure the bindings for the match
     */
    private void configureBindings() {

        BooleanSupplier hubAboutToActivate = () -> {
            double time = DriverStation.getMatchTime();

            return Constants.HubTiming.isActive(time - 5);
        };

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));

        driverController.rightBumper().onTrue(drivetrain.resetGyro());

        driverController.a().onTrue(new InstantCommand(() -> cancelAutomations = !cancelAutomations));

        driverController.leftBumper().whileTrue(new StartEndCommand(
                () -> intake.setRollerDutyCycle(-0.5),
                intake::stopIntakeRoller,
                intake
        ));

        var intakeCommand = Sequences.intakeOpenStart(intake)
                .alongWith(new DriveIntakeCommand(drivetrain, driverController));

        driverController.leftTrigger(0.2).whileTrue(intakeCommand)
                .onFalse(new InstantCommand(() -> {
                    cancelAutomations = true;
                    //currentIntakeMode = IntakeMode.ThirtyBalls;
                }));




       driverController.x().whileTrue(Sequences.delivery(drivetrain, shooter, intake, driverController,
        operatorController.y(), () -> currentIntakeMode));

        //operatorController.a().whileTrue(ShootCommand.shootCommandFactory(shooter, drivetrain, driverController, intake, vision, operatorController.y(), () -> currentIntakeMode));


        driverController.y().toggleOnTrue(new DriveAndHomeToHubCommand(drivetrain, driverController));

        //autoShoot.whileTrue(Commands.print("autoshoot"));


        new Trigger(hubAboutToActivate).onTrue(new InstantCommand(
                () -> {
                    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                    Logger.recordOutput("IsRumbling", true);
                })
                .andThen(new WaitCommand(1.5), new InstantCommand(
                    () -> {
                        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        Logger.recordOutput("IsRumbling", false);
                    }
                )));
    
    }

    private void configureOperatorBindings() {
        operatorController.povUp().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addHoodAngleOffset(ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK))
        );

        operatorController.povDown().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addHoodAngleOffset(ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK.unaryMinus()))
        );

        operatorController.povRight().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addflyWheelOffset(ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK))
        );

        operatorController.povLeft().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addflyWheelOffset(-ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK))
        );

        operatorController.rightBumper().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addRobotAngleOffset(ShooterConstants.ROBOT_ANGLE_OFFSET_PER_CLICK.unaryMinus()))
        );

        operatorController.leftBumper().onTrue(new InstantCommand(() ->
                ShotCalculator.getInstance().addRobotAngleOffset(ShooterConstants.ROBOT_ANGLE_OFFSET_PER_CLICK))
        );

        operatorController.leftStick().onTrue(new InstantCommand(() -> ShotCalculator.getInstance().resetOffsets()));

        operatorController.x().toggleOnTrue(IntakeFactory.resetIntake(intake));

        var shootCommand = ShootCommand.operatorShootCommandFactory(
                                shooter, drivetrain, vision, intake, operatorController.y() ,() -> currentIntakeMode,
                                () -> overrideShooting, operatorController.b());

        var immediateShootCommand = ShootCommand.operatorShootCommandFactory(
                                shooter, drivetrain, vision, intake, operatorController.y() ,() -> currentIntakeMode,
                                () -> overrideShooting, operatorController.b()).onlyIf(()-> !shootCommand.isScheduled());
        
        operatorController.b().onTrue(new CloseCommand(intake).onlyIf(() -> 
                !shootCommand.isScheduled() && 
                !immediateShootCommand.isScheduled()
        ));
        
        operatorController.rightStick().toggleOnTrue(
                new StartEndCommand(
                        () -> overrideShooting = true,
                        () -> overrideShooting = false)
                .alongWith(
                        immediateShootCommand));

        operatorController.a().whileTrue(shootCommand);

        operatorController.start().onTrue(new InstantCommand(() -> HubTiming.setHumanActiveFirst(true)).ignoringDisable(true));

        operatorController.back().onTrue(new InstantCommand(() -> HubTiming.setHumanActiveFirst(false)).ignoringDisable(true));

        operatorController.leftTrigger(0.3).onTrue(
                new InstantCommand(() -> currentIntakeMode = currentIntakeMode.getPrev()));
        
        operatorController.rightTrigger(0.3).onTrue(
                new InstantCommand(() -> currentIntakeMode = currentIntakeMode.getNext()));

        operatorController.y().whileTrue(new StartEndCommand(() -> shooter.getIndexer().spinBackward(),
         () -> shooter.getIndexer().spinForward(), shooter.getIndexer())
         .onlyIf(() -> shooter.getIndexer().getCurrentCommand() == null));



    }

    /**
     * @return the chosen autonomous command.
     */
    public Command getAutonomousCommand() {
        var selectedAuto = autoChooser.get();

        Command autoCommand =
                selectedAuto
                .andThen(drivetrain.idle());

        CommandScheduler.getInstance().removeComposedCommand(selectedAuto);

        return autoCommand.withName(selectedAuto.getName());
    }

    /**
     * Displays the path the auto {@code command} takes
     *
     * @param command the command runnning in auto
     */
    private void displayChosenAuto(Command command) {
        if (RobotState.isEnabled()) {
            drivetrain.clearFiledPath();
            return;
        }

        List<PathPlannerPath> auto;

        try {
            auto = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
        } catch (IOException | ParseException e) {
            Logger.recordOutput("autoDisplay", e.getMessage());
            drivetrain.clearFiledPath();
            return;
        }

        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : auto) {
            path = ChassisConstants.shouldFlipPath() ? path : path.flipPath();
            poses.addAll(path.getPathPoses());
        }

        drivetrain.addPathToField(poses);
    }


    /**
     * @param driveAndHomeToHubCommand that pathplanner will use (replaces it with a PPController)
     * @return A LoggedDashboardChooser for the auto commands and gives
     * PathPlanner sequences for our auto commands
     */
    public LoggedDashboardChooser<Command> registerNamedCommand(DriveAndHomeToHubCommand driveAndHomeToHubCommand) {

        Command aimRobot = new StartEndCommand(() -> {
            driveAndHomeToHubCommand.resetThetaController();
            PPController.setThetaOverride(driveAndHomeToHubCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeToHubCommand.calculateThetaPID())), drivetrain)
                .beforeStarting(driveAndHomeToHubCommand::resetThetaController);


        NamedCommands.registerCommand("shootAndAimMoving",
                ((new ShootCommand(shooter, drivetrain, vision, intake,  () -> false, () -> currentIntakeMode, 
                () -> false, () -> false))
                .alongWith(new ShakeItOffCommand(intake))).beforeStarting(new SpinUp(shooter, drivetrain))
                .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                ((new ShootCommand(shooter, drivetrain, vision, intake, () -> false, () -> currentIntakeMode, 
                () -> false, () -> false))
                        .alongWith(new TwoInOneOut(intake), aimRobotStationary)));

        NamedCommands.registerCommand("spinUp", new RunCommand(() -> shooter.spinUp(17), shooter));

        NamedCommands.registerCommand("waitUntilNoBalls", ObjectDetection.getInstance().waitUntilNoBalls()
                .andThen(Commands.print("no balls")));

        NamedCommands.registerCommand("openIntake",
                new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        NamedCommands.registerCommand("closeIntake",
                new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        NamedCommands.registerCommand("openIntakeAndReset",
                Sequences.intakeOpenStart(intake).beforeStarting(new InstantCommand(() -> intake.resetOpeningMotorEncoder(0))));


        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
        autoChooser.onChange(this::displayChosenAuto);
        autoChooser.addDefaultOption("idle", drivetrain.idle());
        return autoChooser;
    }
}
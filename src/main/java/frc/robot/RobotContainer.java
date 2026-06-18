// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drivetrain.*;
import frc.robot.subsystems.UpdateWigdets.UpdateWidgets;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.IsrIntake.IsrIntake;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class RobotContainer {
    private static RobotContainer instance;

    private final CommandXboxController driverController;

    public final Drivetrain drivetrain;

    private IsrIntake intake;

    private final LoggedDashboardChooser<Command> autoChooser;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer() {
        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        intake = new IsrIntake();

        driverController = new CommandXboxController(0);

        autoChooser = registerNamedCommand(new DriveAndHomeToHubCommand(drivetrain, driverController));

        new UpdateWidgets();

        configureBindings();
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

        driverController.start().onTrue(drivetrain.resetGyro());
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

        // Command aimRobot = new StartEndCommand(() -> {
        //     driveAndHomeToHubCommand.resetThetaController();
        //     PPController.setThetaOverride(driveAndHomeToHubCommand::calculateThetaPID);
        // }, PPController::clearThetaOverride);

        // Command aimRobotStationary = new FunctionalCommand(
        //         driveAndHomeToHubCommand::resetThetaController,
        //         () -> drivetrain.drive(
        //                 new ChassisSpeeds(0, 0,
        //                         driveAndHomeToHubCommand.calculateThetaPID())),
        //         (interrupted) -> drivetrain.stop(),
        //         () -> false,
        //         drivetrain);


        // NamedCommands.registerCommand("shootAndAimMoving",
        //         ((new ShootCommand(shooter, drivetrain, vision, intake, () -> false, () -> currentIntakeMode,
        //                 () -> false, () -> false))
        //                 .alongWith(new ShakeItOffCommand(intake))).beforeStarting(new SpinUp(shooter, drivetrain))
        //                 .alongWith(aimRobot));

        // NamedCommands.registerCommand("shootAndAimStationary",
        //         ((new ShootCommand(shooter, drivetrain, vision, intake, () -> false, () -> currentIntakeMode,
        //                 () -> false, () -> false))
        //                 .alongWith(new TwoInOneOut(intake), aimRobotStationary)));

        // NamedCommands.registerCommand("spinUp", new RunCommand(() -> shooter.spinUp(17), shooter));

        // NamedCommands.registerCommand("waitUntilNoBalls", ObjectDetection.getInstance().waitUntilNoBalls()
        //         .andThen(Commands.print("no balls")));

        // NamedCommands.registerCommand("openIntake",
        //         new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        // NamedCommands.registerCommand("closeIntake",
        //         new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        // NamedCommands.registerCommand("openIntakeAndReset",
        //         Sequences.intakeOpenStart(intake).beforeStarting(new InstantCommand(() -> intake.resetOpeningMotorEncoder(0))));


        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
        autoChooser.onChange(this::displayChosenAuto);
        autoChooser.addDefaultOption("idle", drivetrain.idle());
        return autoChooser;
    }
}
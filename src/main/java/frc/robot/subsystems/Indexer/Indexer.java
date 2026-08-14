package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOReplay;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.utils.ParallelRaceGroupWithWinner;

public class Indexer extends SubsystemBase {

    // What modes (states) does the Indexer have
    public enum IndexerMode{
        BACKWARD,
        STOPPED,
        FORWARD
    }

    // Hardware abstraction layer (IO interface)
    private final IndexerIO io;

    // Inputs object for advantage kit (IO Interface)
    private IndexerInputsAutoLogged inputs;

    public Indexer(){
        // Based on the robot's mode use the relevant hardware abstraction
        switch (Constants.currentMode) {
            case REAL -> io = new IndexerIOReal(); // Real robot

            case SIM -> io = new IndexerIOSim(); // Simulation Robot
        
            default -> io = new IndexerIOReplay(); // Replay Robot (advantage kit)
        }
        
        inputs = new IndexerInputsAutoLogged();

        Logger.recordOutput("Indexer/Mode", IndexerMode.STOPPED); // Log that the indexer starts disabled
    }

    @Override
    public void periodic() {
        io.update(inputs); // Read the inputs from the IO interface
        Logger.processInputs(getName(), inputs); // Advantage kit does it's magic
    }

    /**
     * Enables the indexer
     */
    public void spinForward(){
        io.spinForward(); // Tell the hardware abstraction to spin forward
        Logger.recordOutput("Indexer/Mode", IndexerMode.FORWARD); // Log that the indexer is now moving forward
    }

    /**
     * Enables the indexer in reverse
     */
    public void spinBackward(){
        io.spinBackward(); // Tell the hardware abstraction to spin backward
        Logger.recordOutput("Indexer/Mode", IndexerMode.BACKWARD); // Log that the indexer is now moving backwards
    }

    /**
     * Disable the indexer
     */
    public void turnOff(){
        io.turnOff(); // Tell the hardware abstraction to stop
        Logger.recordOutput("Indexer/Mode", IndexerMode.STOPPED); // Log that the indexer is now stopped
    }

    /**
     * @return whether or not the indexer is stuck
     */
    public boolean isStuck(){
        return inputs.isStuck; // Never have a getter in the io interface, always read through the inputs object
    }

    /**
     * @return a command which disables the indexer
     */
    public Command turnOffIndexerCommand(){
        return new InstantCommand(this::turnOff, this); // Turns off the indexer
    }

    /**
     * @return A command which enables the indexer, waits long enough to be sure it is stuck, and if it is stuck disable it.
     */
    public Command turnOnIndexerCommand(){
        @SuppressWarnings("resource") // Bye bye yellow squiggly
        Alert failedToTurnOn = new Alert("Failed to turn on Indexer!", AlertType.kError);

        /*
         * At any point in time the Indexer may either be stuck or not stuck:
         * If the indexer isn't stuck, wait until it is
         * If the indexer is stuck, wait for 0.1 seconds and then you are sure that the indexer is actually stuck
         * If the indexer is stuck and after 0.1 seconds it is no longer stuck, wait until it is stuck.
         * (Ew + use state machines + L + ratio + -1000 aura)
         */
        Command waitUntilStuck = ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
            new WaitUntilCommand(() -> !isStuck()),

            ShootRealConstants.TIME_TO_NOT_BE_DEADBAND,

            new WaitUntilCommand(this::isStuck));


        return new ParallelRaceGroupWithWinner(
            new InstantCommand(this::spinForward) // Start spinning forward

                .andThen(new WaitUntilCommand(() -> !isStuck()), // Once the indexer ISN'T stuck
                
                new InstantCommand(() -> failedToTurnOn.set(false))), // Make sure that the alert that the indexer is stuck is off

            waitUntilStuck // Or stop when the indexer is stuck

        ).andThenOnlyIfWinner(waitUntilStuck, new InstantCommand(this::turnOff) // If the indexer is stuck, turn it off

        .andThen(new InstantCommand(() -> failedToTurnOn.set(true)))); // and notify the driver
    }
}

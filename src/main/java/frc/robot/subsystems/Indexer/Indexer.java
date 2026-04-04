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

    private final IndexerIO io;

    private IndexerInputsAutoLogged inputs;

    public Indexer(){
        switch (Constants.currentMode) {
            case REAL -> io = new IndexerIOReal();

            case SIM -> io = new IndexerIOSim();
        
            default -> io = new IndexerIOReplay();
        }
        
        inputs = new IndexerInputsAutoLogged();

        Logger.recordOutput("Indexer/Mode", IndexerIO.IndexerMode.STOPPED);
    }

    @Override
    public void periodic() {
        io.update(inputs);
        
        Logger.processInputs(getName(), inputs);
    }

    /**
     * Enables the indexer
     */
    public void spinForward(){
        io.spinForward();
        Logger.recordOutput("Indexer/Mode", IndexerIO.IndexerMode.FORWARD);
    }

    /**
     * Enables the indexer in reverse
     */
    public void spinBackward(){
        io.spinBackward();
        Logger.recordOutput("Indexer/Mode", IndexerIO.IndexerMode.BACKWARD);
    }

    /**
     * Diisable the indexer
     */
    public void turnOff(){
        io.turnOff();
        Logger.recordOutput("Indexer/Mode", IndexerIO.IndexerMode.STOPPED);
    }

    /**
     * @return A command which enables the indexer, waits long enough to be sure it is stuck, and if it is stuck disable it.
     */
    public Command turnOnIndexerCommand(){
        @SuppressWarnings("resource")
        Alert failedToTurnOn = new Alert("Failed to turn on Indexer!", AlertType.kError);

        Command waitUntilStuck = ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
            new WaitUntilCommand(() -> !isStuck()),

            ShootRealConstants.TIME_TO_NOT_BE_DEADBAND, 

            new WaitUntilCommand(() -> isStuck()));


        return new ParallelRaceGroupWithWinner(
            new InstantCommand(this::spinForward)

                .andThen(new WaitUntilCommand(() -> !isStuck()),
                
                new InstantCommand(() -> failedToTurnOn.set(false))),

            waitUntilStuck

        ).andThenOnlyIfWinner(waitUntilStuck, new InstantCommand(this::turnOff)

        .andThen(new InstantCommand(() -> failedToTurnOn.set(true))));
    }

    /**
     * @return a command which disables the indexer
     */
    public Command turnOffIndexerCommand(){
        return new InstantCommand(() -> turnOff(), this);
    }

    /**
     * @return whether or not the indexer is stuck
     */
    public boolean isStuck(){
        return inputs.isStuck;
    }
}

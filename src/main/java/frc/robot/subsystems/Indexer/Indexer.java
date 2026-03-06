package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.utils.ErrorMessage;
import frc.utils.ParallelRaceGroupWithWinner;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;

    private IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();


    public Indexer(){
        this.io = RobotBase.isReal()? new IndexerIOReal(): new IndexerIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
        
        Logger.processInputs(getName(), inputs);
    }

    /**
     * sets the spindexer motor precent to the constant
     */
    public void turnOn(){
        io.turnOn();
    }

    /**
     * stops the motor
     */
    public void turnOff(){
        io.turnOff();
    }

    /**
     * @return command which turns on the indexer
     */
    public Command turnOnIndexerCommand(){
        @SuppressWarnings("resource")
        Alert failedToTurnOn = new Alert("Failed to turn on Indexer!", AlertType.kError);

        Command waitUntilStuck = ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
            new WaitUntilCommand(() -> !isStuck()),

            ShootRealConstants.TIME_TO_NOT_BE_DEADBAND, 

            new WaitUntilCommand(() -> isStuck()));


        return new ParallelRaceGroupWithWinner(
            new InstantCommand(this::turnOn)

                .andThen(new WaitUntilCommand(() -> !isStuck()),
                
                new InstantCommand(() -> failedToTurnOn.set(false))),

            waitUntilStuck

        ).andThenOnlyIfWinner(waitUntilStuck, new InstantCommand(this::turnOff)

        .andThen(new InstantCommand(() -> failedToTurnOn.set(true))));
    }

    /**
     * @return command which turns off the indexer
     */
    public Command turnOffIndexerCommand(){
        return new InstantCommand(() -> turnOff(), this);
    }

    /**
     * @return if the indexer is stuck
     */
    public boolean isStuck(){
        return io.isStuck();
    }
}

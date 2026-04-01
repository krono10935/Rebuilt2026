package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.utils.ParallelRaceGroupWithWinner;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;

    private IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    public Indexer(){
        this.io = RobotBase.isReal() ? new IndexerIOReal(): new IndexerIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
        
        Logger.processInputs(getName(), inputs);
    }

    /**
     * Enables the indexer
     */
    public void turnOn(){
        io.turnOn();
    }

    /**
     * Enables the indexer in reverse
     */
    public void reverse(){
        io.reverse();
    }

    /**
     * Diisable the indexer
     */
    public void turnOff(){
        io.turnOff();
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
            new InstantCommand(this::turnOn)

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
        if (inputs.controlMode == ControlMode.STOP || inputs.targetSpeedMPS == 0){
            return false;
        }
        else{
            return (Math.abs(inputs.motorLeftSpeedMPS) < IndexerConstants.SPEED_DEADBAND ||
            Math.abs(inputs.motorRightSpeedMPS) < IndexerConstants.SPEED_DEADBAND);
        }

        
    }

    public void setSpeed(double rps){
        io.setSpeed(rps);
    }
}

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.ErrorMessage;

public class Indexer extends SubsystemBase implements ErrorMessage.ErrorSender {

    private final IndexerIO io;

    private IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    private boolean failedToStop;

    public Indexer(){
        this.io = RobotBase.isReal()? new IndexerIOReal(): new IndexerIOSim();

        failedToStop = false;

        ErrorMessage.create(this,
                "error closing" + this.getName(),
                () -> failedToStop);
    }

    @Override
    public void periodic() {
        io.update(inputs);
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
     * @return command which turn's on the indexer
     */
    public Command turnOnIndexerCommand(){
        return new InstantCommand(() -> turnOn(), this);
    }

    /**
     * @return command which turn's off the indexer
     */
    public Command turnOffIndexerCommand(){
        return new InstantCommand(() -> turnOff(), this);
    }

    @Override
    public void send(boolean shouldDisplayError, int code){
        failedToStop = shouldDisplayError;
    }
}

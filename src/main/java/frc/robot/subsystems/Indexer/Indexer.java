package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;

    private IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    public Indexer(){
        this.io = RobotBase.isReal()? new IndexerIOReal(): new IndexerIOSim();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public void start(){
        io.start();
    }
    public void stop(){
        io.stop();
    }
}

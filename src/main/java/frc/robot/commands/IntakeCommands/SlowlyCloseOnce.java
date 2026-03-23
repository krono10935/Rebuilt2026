package frc.robot.commands.IntakeCommands;


import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class SlowlyCloseOnce extends SlowlyClose {

    private boolean hasClosed; 

    public SlowlyCloseOnce(Intake intake) {
        super(intake);
        hasClosed = false;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || hasClosed;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        super.execute();

        if (openPos != IntakeConstants.OPEN_POSITION){
            hasClosed = true;
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setPositionMotorSlowly(0);

    }


}

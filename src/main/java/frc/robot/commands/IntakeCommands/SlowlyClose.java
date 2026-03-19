package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SlowlyClose extends Command {

    protected final Intake intake;

    private final LoggedNetworkNumber slowlyClosePercent;

    protected double openPos = IntakeConstants.OPEN_POSITION;

    private boolean closing = true;

    private Timer timerToOpenAgain;

    public SlowlyClose(Intake intake) {
        this.intake = intake;
        slowlyClosePercent = new LoggedNetworkNumber("SlowlyClose/percent", 0 );
        openPos = IntakeConstants.OPEN_POSITION;
        timerToOpenAgain = new Timer();
        addRequirements(intake);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intake.setPositionMotorPercent(0);
    }

    @Override
    public void execute() {
        super.execute();

//        if((intake.getPositionMotorVelocity() <= 0.001 && closing) && timerToOpenAgain.get() > 1.5){
//            openPos /=2.0;
//            intake.setPosition(openPos);
//            closing = false;
//            timerToOpenAgain.reset();
//
//        }
        if(Math.abs(intake.getIntakePosition()) <= 0.003  ){
            openPos /=2.0;
            intake.setPosition(openPos);
            closing = false;
        }
        if(closing && Math.abs(intake.getIntakePosition() - openPos) <= 0.003  ){
            closing = true;
            intake.setPositionMotorSlowly(0);
            timerToOpenAgain.reset();
            timerToOpenAgain.start();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setPositionMotorSlowly(0);
        intake.setPercent(-0.2);

    }


}

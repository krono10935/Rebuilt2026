package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.littletonrobotics.junction.Logger;

public class IsrIntake extends SubsystemBase {
    public enum PositionState {
        CLOSED,
        OPEN;
    }

    public enum RollerState {
        OFF,
        FORWARD,
        REVERSE
    }

    private final IsrIntakeIO io;
    private final IsrIntakeInputsAutoLogged isrIntakeInputs;

    private PositionState positionState;
    private RollerState rollerState;

    public IsrIntake(){

        io = RobotBase.isReal()? new IsrIntakeIOReal() : new IsrIntakeIOSim();

        positionState = PositionState.CLOSED;

        rollerState = RollerState.OFF;

        isrIntakeInputs = new IsrIntakeInputsAutoLogged();
    }

    private Rotation2d getIsrIntakePosition(){ return isrIntakeInputs.positionMotorAngle; }

    private void setIsrPositionMotorDutyCycle(double dutyCycle){ io.setIsrRollerMotorDutyCycle(dutyCycle);}

    private void moveToPosition(Rotation2d angle){
        io.moveToPosition(angle);
    }

    private void setIsrRollerDutyCycle(double dutyCycle){
        io.setIsrRollerMotorDutyCycle(dutyCycle);
    }

    private void stopIsrRollerMotor(){
        io.stopIsrRollerMotor();
    }

    private boolean isIsrPositionMotorAtSetPoint(){
        return isrIntakeInputs.isPositionMotorAtSetPoint;
    }

    private void setPositionState(PositionState positionState){
        this.positionState = positionState;
        switch (positionState) {
            case CLOSED:
                moveToPosition(IsrIntakeConstants.CLOSED_POS);
                break;

            case OPEN:
                moveToPosition(IsrIntakeConstants.OPEN_POS);
                break;
        
            default:
                break;
        }
    }

    public Command setPositionStateCommand(PositionState positionState){
        return new InstantCommand(() -> {
            setPositionState(positionState); //TODO test timing issues
        }).andThen(
            new WaitUntilCommand(this::isIsrPositionMotorAtSetPoint)
        ).withName("SetPositionState: " + positionState.name());
    }

    private void setRollerState(RollerState rollerState){
        this.rollerState = rollerState;
        switch (rollerState) {
            case OFF:
                stopIsrRollerMotor();
                break;

            case FORWARD:
                setIsrRollerDutyCycle(IsrIntakeConstants.ROLLER_DUTYCYCLE);
                break;

            case REVERSE:
                setIsrRollerDutyCycle(-IsrIntakeConstants.ROLLER_DUTYCYCLE);
                break;
        
            default:
                break;
        }
    }

    public Command setRollerStateCommand(RollerState rollerState){
        return new InstantCommand(() -> {
            setRollerState(rollerState); //TODO: test timing issues
        }).andThen(
            new WaitUntilCommand(() -> isrIntakeInputs.isRollerMotorAtSetPoint)
        ).withName("SetRollerState: " + rollerState.name());
    }

    public RollerState getRollerState(){
        return rollerState;
    }

    public PositionState getPositionState(){
        return positionState;
    }

    public void periodic(){
        io.updateInputs(isrIntakeInputs);

        Logger.processInputs(getName(), isrIntakeInputs);

        String currCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();
        Logger.recordOutput("IsrIntake/Current Command", currCommand);

        Logger.recordOutput("IsrIntake/RollerState", rollerState.name());
        Logger.recordOutput("IsrIntake/PositionState", positionState.name());
    }


}

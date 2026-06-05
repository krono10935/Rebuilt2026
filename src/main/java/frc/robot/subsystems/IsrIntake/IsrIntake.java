package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IsrIntake extends SubsystemBase {
    private final IsrIntakeIO io;
    private final IsrIntakeInputsAutoLogged isrIntakeInputs;

    public IsrIntake(){

        io = RobotBase.isReal()? new IsrIntakeIOReal() : new IsrIntakeIOSim();

        isrIntakeInputs = new IsrIntakeInputsAutoLogged();
    }

    public double getIsrIntakePosition(){ return isrIntakeInputs.positionMotorMeters; }

    public void setIsrPositionMotorDutyCycle(double DutyCycle){ io.setIsrRollerMotorDutyCycle(DutyCycle);}

    public void moveToPosition(double positionMeters){
        io.moveToPosition(positionMeters);
    }

    public void setIsrRollerDutyCycle(double DutyCycle){
        io.setIsrRollerMotorDutyCycle(DutyCycle);
    }

    public void stopIsrRollerMotor(){
        io.stopIsrRollerMotor();
    }

    public boolean isIsrPositionMotorAtSetPoint(){
        return isrIntakeInputs.isPositionMotorAtSetPoint;
    }

    public void periodic(){
        io.updateInputs(isrIntakeInputs);

        Logger.processInputs(getName(), isrIntakeInputs);

        String currCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();
        Logger.recordOutput("IsrIntake/Current Command ", currCommand);
    }


}

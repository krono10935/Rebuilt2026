package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IsrIntake extends SubsystemBase {
    private final IsrIntakeIO io;
    private final IsrIntakeInputsAutoLogged isrIntakeInputs;

    public IsrIntake(){

        io = RobotBase.isReal()? new IsrIntakeIOReal() : new IsrIntakeIOSim();

        isrIntakeInputs = new IsrIntakeInputsAutoLogged();
    }

    public double getIsrIntakePosition(){ return isrIntakeInputs.positionMotorMeters; }

    public void setIsrPositionMotorDutyCycle(double dutyCycle){ io.setIsrRollerMotorDutyCycle(dutyCycle);}

    public void moveToPosition(double positionMeters){
        io.moveToPosition(positionMeters);
    }

    public void setIsrRollerDutyCycle(double dutyCycle){
        io.setIsrRollerMotorDutyCycle(dutyCycle);
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
        Logger.recordOutput("IsrIntake/Current Command", currCommand);
    }


}

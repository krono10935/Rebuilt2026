package frc.robot.subsystems.IsrIntake;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IsrIntakeIOReal {
    private final BasicMotor isrRollerMotor;
    private final BasicMotor isrPositionMotor;

    public IsrIntakeIOReal(){

        isrRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrRollerMotorConfig);
        isrPositionMotor = new BasicSparkFlex(IsrIntakeConstants.isrPositionMotorConfig);


    }


}

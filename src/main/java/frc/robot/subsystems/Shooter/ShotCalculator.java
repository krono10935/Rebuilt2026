package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

public class ShotCalculator {
    private static ShotCalculator instance;

    private final LinearFilter robotAngleFIlter = 
        LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));
}

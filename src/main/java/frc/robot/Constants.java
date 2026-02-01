package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivetrain.constants.ChassisType;

public class Constants {

    public static final ChassisType CHASSIS_TYPE = ChassisType.COMPBOT;

    public static final double LOOP_PERIOD_SECONDS = 0.02;

    public enum Phase{
            AUTO(0 ,0),
            TranistionShift(140,130),
            FirstShift(130,105),
            SecondShift(105,80),
            ThirdShift(80,55),
            FourthShift(55,30),
            EndGame(30,0),
            Invalid(0,0);


            public final double STARTING_TIME;
            public final double FINSIHING_TIME;
            
            Phase(double StartingTime, double FinishingTime){
                STARTING_TIME = StartingTime;
                FINSIHING_TIME = FinishingTime;
            }

            public static Phase getActivePhase(double time){
                if(RobotState.isAutonomous()) return AUTO;

                for (Phase phase : Phase.values()){
                    if ((time <= phase.STARTING_TIME) && (time >= phase.FINSIHING_TIME)){
                        return phase;
                    }
                }

                return Invalid;
            }
    }

    public static class HubTiming{
        
        private static boolean isActiveFirst;

        /**
         * 
         * @param team Get from driverstations game specific message the team
         * @param alliance Get from driverstation
         */
        public static void setStartingTeam(String team, Alliance alliance){
            boolean isRed = alliance == Alliance.Red; 

            if (team == "R" && isRed || team == "B" && !isRed){
                isActiveFirst = false;
            } else {
                isActiveFirst = true;
            }
        }

        /**
         * 
         * @param time time to check if the hub would be activated during
         * @return whether or not at a given timestamp the hub would be active for your team
         */
        public static boolean isActive(double time){
            Phase phase = Phase.getActivePhase(time);

            switch (phase) {
                case AUTO,TranistionShift,EndGame:
                    return true;
                case SecondShift,FourthShift:
                    // If red is inactive first and we are red, then we will be active shifts 2,4
                    // Otherwise we will be inactive in those Shifts
                    return isActiveFirst;
                case FirstShift,ThirdShift:
                    return !isActiveFirst;
                default:
                    return false;
            }
            
        }

    }
}
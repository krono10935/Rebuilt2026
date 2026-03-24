package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivetrain.constants.ChassisType;
import org.littletonrobotics.junction.Logger;

public class Constants {

    public static final ChassisType CHASSIS_TYPE = ChassisType.COMPBOT;

    public static final boolean USE_OBJECT_DETECTION = true;

    public static final double LOOP_PERIOD_SECONDS = 0.02;

    public static final double ALL_SUBSYSTEMS_MAX_CLOSING_TIME = 1.0;

    public static final double HUB_ACTIVITY_DEABAND_AFTER_ACTIVE = 3;

    public static final double HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE = 2;

    public static final ProfiledPIDController THETA_CONTROLLER = 
        new ProfiledPIDController(4,4,0,
        new Constraints(10, 5));

    static{
        THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        THETA_CONTROLLER.setIntegratorRange(-10, 10);
        THETA_CONTROLLER.setIZone(Rotation2d.fromDegrees(20).getRadians());
        THETA_CONTROLLER.setTolerance(Rotation2d.fromDegrees(2).getRadians());
    }

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
        
        private static Boolean isActiveFirst = null;
        private static Boolean isActiveFirstHuman = null;

        /**
         * 
         * @param team Get from driverstations game specific message the team
         * @param alliance Get from driverstation
         */
        public static void setStartingTeam(String team, Alliance alliance){
            boolean isRed = alliance == Alliance.Red;

            if ((team.charAt(0) == 'R' && isRed) || (team.charAt(0) == 'B' && !isRed)){
                isActiveFirst = false;
            } else {
                isActiveFirst = true;
            }

            Logger.recordOutput("isActiveFirstFMS", isActiveFirst);
        }

        public static Boolean getAutoIsActiveDetection(){
            return isActiveFirst;
        }

        public static void setHumanActiveFirst(boolean isActiveFirstHumanInput){
            isActiveFirstHuman = isActiveFirstHumanInput;
            Logger.recordOutput("isActiveFirstHuman", isActiveFirstHuman);
        }

        public static boolean isActiveFirst(){
            if (isActiveFirst != null) return isActiveFirst;
            if (isActiveFirstHuman != null) return isActiveFirstHuman;
            return true;
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
                    return isActiveFirst();
                case FirstShift,ThirdShift:
                    return !isActiveFirst();
                default:
                    return false;
            }
            
        }

        /**
         * 
         * @param time time to check if the hub would be activated during
         * @param timeBefore time before the timestamp to check activity
         * @param timeAfter time after the timestamp to check activity
         * @return whether or not at a given timestamp the hub would be active for your team
         */
        public static boolean isActiveWithMargin(double time, double timeBefore, double timeAfter){
            return isActive(time + timeBefore) || isActive(time) || isActive(time - timeAfter);
        }
    }
}
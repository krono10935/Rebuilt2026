package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.constants.ChassisType;

public class Constants {

    public static final ChassisType CHASSIS_TYPE = ChassisType.COMPBOT;

    public static final boolean USE_OBJECT_DETECTION = true;

    public static final Mode simMode = Mode.REAL;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final double LOOP_PERIOD_SECONDS = 0.02;

    public static final double ALL_SUBSYSTEMS_MAX_CLOSING_TIME = 1.0;

    public static final double HUB_ACTIVITY_DEABAND_AFTER_ACTIVE = 3;

    public static final double HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE = 2;

    public static final ProfiledPIDController THETA_CONTROLLER =
            new ProfiledPIDController(4, 8, 0.1,
                    new Constraints(10, 5));

    static {
        THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        THETA_CONTROLLER.setIntegratorRange(-10, 10);
        THETA_CONTROLLER.setIZone(Rotation2d.fromDegrees(20).getRadians());
        THETA_CONTROLLER.setTolerance(Rotation2d.fromDegrees(2).getRadians());
        SmartDashboard.putData(THETA_CONTROLLER);
    }

    public enum Phase {
        AUTO(0, 0),
        TranistionShift(140, 130.1),
        FirstShift(130, 105.1),
        SecondShift(105, 80.1),
        ThirdShift(80, 55.1),
        FourthShift(55, 30.1),
        EndGame(30, 0.1),
        Invalid(0, 0);


        public final double STARTING_TIME;
        public final double FINSIHING_TIME;

        Phase(double StartingTime, double FinishingTime) {
            STARTING_TIME = StartingTime;
            FINSIHING_TIME = FinishingTime;
        }

        public double timeToShiftEnd(double time) {
            return time - FINSIHING_TIME;
        }

        public static Phase getActivePhase(double time) {
            if (RobotState.isAutonomous()) return AUTO;

            for (Phase phase : Phase.values()) {
                if ((time <= phase.STARTING_TIME) && (time >= phase.FINSIHING_TIME)) {
                    return phase;
                }
            }

            return Invalid;
        }
    }

    public static class HubTiming {

        private static boolean isActiveFirst = false;
        private static DataSource dataSource = DataSource.CODE;

        public enum DataSource {
            FMS,
            HUMAN,
            CODE;
        }

        static {
            SmartDashboard.putString("active first status", "Not decided");
            SmartDashboard.putBoolean("fms said active first", false);
        }

        /**
         *
         * @param team     Get from driverstations game specific message the team
         * @param alliance Get from driverstation
         */
        public static void setStartingTeam(String team, Alliance alliance) {
            boolean isRed = alliance == Alliance.Red;

            if ((team.charAt(0) == 'R' && isRed) || (team.charAt(0) == 'B' && !isRed)) {
                isActiveFirst = false;
            } else {
                isActiveFirst = true;
            }

            dataSource = DataSource.FMS;
            String text = isActiveFirst ? "We lost" : "We won";
            SmartDashboard.putBoolean("fms said active first", true);
            SmartDashboard.putString("active first status", text);
        }

        public static Boolean getAutoIsActiveDetection() {
            return isActiveFirst;
        }

        public static void setHumanActiveFirst(boolean isActiveFirstHumanInput) {
            if (dataSource == DataSource.FMS)
                return;

            dataSource = DataSource.HUMAN;
            isActiveFirst = isActiveFirstHumanInput;
            String text = isActiveFirst ? "We lost" : "We won";
            SmartDashboard.putString("active first status", text);
        }

        public static double timeRemainingToShift(double time) {
            Phase phase = Phase.getActivePhase(time);
            return time - phase.FINSIHING_TIME;
        }

        public static boolean isActiveFirst() {
            return isActiveFirst;
        }

        public static double timeToNextShift(double time) {
            return Phase.getActivePhase(time).timeToShiftEnd(time);
        }

        /**
         *
         * @param time time to check if the hub would be activated during
         * @return whether or not at a given timestamp the hub would be active for your team
         */
        public static boolean isActive(double time) {
            Phase phase = Phase.getActivePhase(time);
            return switch (phase) {
                case AUTO, TranistionShift, EndGame -> true;
                case SecondShift, FourthShift ->
                    // If red is inactive first and we are red, then we will be active shifts 2,4
                    // Otherwise we will be inactive in those Shifts
                        isActiveFirst();
                case FirstShift, ThirdShift -> !isActiveFirst();
                default -> false;
            };

        }

        /**
         *
         * @param time       time to check if the hub would be activated during
         * @param timeBefore time before the timestamp to check activity
         * @param timeAfter  time after the timestamp to check activity
         * @return whether or not at a given timestamp the hub would be active for your team
         */
        public static boolean isActiveWithMargin(double time, double timeBefore, double timeAfter) {
            return isActive(time + timeBefore) || isActive(time) || isActive(time - timeAfter);
        }
    }
}
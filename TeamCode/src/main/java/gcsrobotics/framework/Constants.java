package gcsrobotics.framework;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import gcsrobotics.framework.hardware.GoBildaPinpointDriver;

/// <strong>A class that holds all of the robot constants</strong>
@Config("Robot Constants") //This Config tag allows for these values to be tuned in real time using the dashboard
public class Constants {
    public static double KpDrive;
    public static double KdDrive;
    public static double KpHeadingCorrection;
    public static double KpTurn;

    public static double PATH_TOLERANCE_IN;
    public static double CHAIN_TOLERANCE_IN;
    public static double TURN_TOLERANCE_DEG;
    public static double PATH_SETTLE_TIME_MS;
    public static double TURN_SETTLE_TIME_MS;
    public static double PATH_TIMEOUT_MS;
    public static double MAX_HEADING_CORRECTION_POWER;

    public static double GLOBAL_DEFAULT_MOTOR_POWER;
    public static int ENCODER_TOLERANCE;
    public static double AUTO_MAX_POWER;

    public static DcMotorSimple.Direction flDirection;
    public static DcMotorSimple.Direction frDirection;
    public static  DcMotorSimple.Direction blDirection;
    public static DcMotorSimple.Direction brDirection;

    public static GoBildaPinpointDriver.EncoderDirection xPodDirection;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection;

    public static double X_ODO_POD_OFFSET_MM;
    public static double Y_ODO_POD_OFFSET_MM;

    static{

        /* Examples of constants
        --- Use this to set things like servo and motor positions ---
        This allows you to change numbers in only one spot if you need
        to make an adjustment

        Add any other constants you need here and use them throughout your opmodes!
         */

        // PATHING CONSTANTS - Likely necessary to tune

        KpDrive = 0.01;
        KdDrive = 0.001;
        KpHeadingCorrection = 0.02; //P value for heading correction during paths
        KpTurn = 0.1;

        // PATHING CONSTANTS - Likely won't need to tune
        PATH_TOLERANCE_IN = 1;
        CHAIN_TOLERANCE_IN = 3;
        TURN_TOLERANCE_DEG = 1.5;
        PATH_SETTLE_TIME_MS = 100;
        TURN_SETTLE_TIME_MS = 150;
        PATH_TIMEOUT_MS = 3000;
        MAX_HEADING_CORRECTION_POWER = 0.3; // Maximum power a heading correction can apply


        // This must be between 0 and 1, and scales the maximum motor power of an auto path or chain
        AUTO_MAX_POWER = 0.6;

        // MOTOR CONSTANTS -- Likely won't need to tune
        ENCODER_TOLERANCE = 10; // How close a motor needs to be to the actual target before considering it "there"
        GLOBAL_DEFAULT_MOTOR_POWER = 1; // Default motor speed of a motor. Almost never need to tune

        // DRIVETRAIN CONSTANTS -- May need to tune once per drivetrain and determines motor directions
        flDirection = DcMotorSimple.Direction.REVERSE;
        frDirection = DcMotorSimple.Direction.FORWARD;
        blDirection = DcMotorSimple.Direction.REVERSE;
        brDirection = DcMotorSimple.Direction.FORWARD;

        // ODOMETRY CONSTANTS -- Determines if an odometry encoder is inverted
        //Reverse this if going forward makes the x coordinate go down
        xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        //Reverse this if going left makes the y coordinate go down
        yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        // You will need to change this for your robot
        X_ODO_POD_OFFSET_MM = 0;
        Y_ODO_POD_OFFSET_MM = 0;
    }
}
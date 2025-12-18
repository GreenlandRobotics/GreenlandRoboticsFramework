package gcsrobotics.framework;

import static gcsrobotics.framework.Constants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Supplier;

import gcsrobotics.framework.hardware.DcMotorEnhanced;

/** The base for all autonomous opmodes. This class extends OpModeBase, so you have access to all
 * hardware, global variables, and more.
 * <br>
 * This provides a centralized and modular approach to autonomous opmodes.<br>
 * This class has pathing methods,simple drive methods, and lots of utility methods
 * @author Josh Kelley
 **/
@SuppressWarnings("all")
public abstract class AutoBase extends OpModeBase {

    //Global variables specific to pidDrivePower
    private double lastXError = 0;
    private double lastYError = 0;
    private ElapsedTime pidTimerX = new ElapsedTime();
    private ElapsedTime pidTimerY = new ElapsedTime();

    protected enum Axis{
        X,
        Y,
        NONE;
    }


    /// Optional method to define when you want to run code in init
    protected void initSequence(){}

    /// The code that runs during start
    protected abstract void runSequence();

    @Override
    protected void runInit(){
        initSequence();
    }

    /// Very important method: run() extends OpMode
    @Override
    protected void run() {
        runSequence();
    }

    private final ElapsedTime stuckTimer = new ElapsedTime();
    private double lastDistanceToTarget = Double.MAX_VALUE;
    private double targetAngle = 0;


    /** Used for making small, corrective movements when you need simple directional movement
     @param direction the direction you want to move
     @param power the power to set the motors to
     @param time the time to wait in milliseconds
    */
    protected void simpleDrive(Axis direction, double power, int time){
        switch(direction){
            //Axis X
            case X:
                for(DcMotorEnhanced motor: new DcMotorEnhanced[]{fl,fr,bl,br}){
                    motor.setPower(power);
                }
                wait(time);
                stopMotors();
                break;
            //Axis Y: Move Left / Right
            case Y:
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                wait(time);
                stopMotors();
                break;
            default:
                //Throw an error if there is no Axis Direction (because Axis is defined as an enum)
                throw new IllegalArgumentException("Invalid direction for simpleDrive: NONE" +
                        "is not a valid direction");
        }


    }

    /// Waits for a set amount of time, similar to sleep, but better because it keeps updating
    /// @param milliseconds the amount of time to wait in milliseconds
    protected void wait(int milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        do {
            telemetry.addLine(String.format("Waiting for %d milliseconds", (milliseconds - timer.milliseconds())));
            telemetry.update();
            //increment by 50
            sleep(50);
        } while((opModeIsActive() && timer.milliseconds() < milliseconds));
    }


    /// <strong>Precise positioning</strong><br>
    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    /// OVERLOADED method (Only takes in X,Y. Then, calls path with NO axis.)
    protected void path(int targetX, int targetY) {
        path(targetX, targetY, Axis.NONE);
    }


    /// <strong>Precise positioning</strong><br>
    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    /// This method has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    /// @param forgiveAxis the axis you want to not consider in the end behavior
    protected void path(int targetX, int targetY, Axis forgiveAxis) {
        ElapsedTime endTimer = new ElapsedTime();
        boolean endSession = false;

        resetPidTimers();

        while (opModeIsActive() && notStuck(targetX,targetY)) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget;
            switch(forgiveAxis){
                case X:
                    atTarget = Math.abs(xError) < PATH_TOLERANCE_IN;
                    break;
                case Y:
                    atTarget = Math.abs(yError) < PATH_TOLERANCE_IN;
                    break;
                default:
                    atTarget = Math.abs(xError) < PATH_TOLERANCE_IN && Math.abs(yError) < PATH_TOLERANCE_IN;
                    break;
            }
            //Toggle logic (Making sure that a session is active when pathing).
            if (atTarget && !endSession) {
                endSession = true;
                endTimer.reset();
            } else if (!atTarget) {
                endSession = false;
            }

            if (endSession && endTimer.milliseconds() > PATH_SETTLE_TIME_MS) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(
                    KpHeadingCorrection * (getAngle() - this.targetAngle),
                    -MAX_HEADING_CORRECTION_POWER, MAX_HEADING_CORRECTION_POWER
            );

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("PATH", xError, yError, xPower, yPower, headingCorrection);
        }

        stopMotors();
    }

    /// <strong>Fast, but not as accurate</strong><br>
    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    protected void chain(int targetX, int targetY) {
        chain(targetX, targetY, Axis.NONE);
    }

    /// <strong>Fast, but not as accurate</strong><br>
    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    /// This method has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    /// @param forgiveAxis the axis you want to not consider in the end behavior
    protected void chain(int targetX, int targetY, Axis forgiveAxis) {
        resetPidTimers();

        while (opModeIsActive() && notStuck(targetX,targetY)) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget;
            switch(forgiveAxis){
                case X:
                    atTarget = Math.abs(xError) < CHAIN_TOLERANCE_IN;
                    break;
                case Y:
                    atTarget = Math.abs(yError) < CHAIN_TOLERANCE_IN;
                    break;
                default:
                    atTarget = Math.abs(xError) < CHAIN_TOLERANCE_IN && Math.abs(yError) < CHAIN_TOLERANCE_IN;
                    break;
            }

            if (atTarget) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(
                    KpHeadingCorrection * (getAngle() - this.targetAngle),
                    -MAX_HEADING_CORRECTION_POWER, MAX_HEADING_CORRECTION_POWER
            );

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("CHAIN", xError, yError, xPower, yPower, headingCorrection);
        }
        stopMotors();
    }

    /**
     * Turns the robot to a specific heading (in degrees).
     * Positive angles are counterclockwise, negative are clockwise.
     * Uses a proportional controller for heading.
     * @param targetAngle the angle to turn to, in degrees (0 = field forward, CCW+)
     */
    protected void turn(double targetAngle) {

        this.targetAngle = targetAngle;

        targetAngle = normalizeAngle(targetAngle);

        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive()) {
            double currentAngle = getAngle();
            double error = -normalizeAngle(targetAngle - currentAngle); //Negative to compensate

            // Check if within tolerance to start settling
            if (Math.abs(error) <= TURN_TOLERANCE_DEG) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                }
                if (settleTimer.milliseconds() >= TURN_SETTLE_TIME_MS) {
                    break; // Done turning
                }
            } else {
                settling = false; // Not settled, reset timer next time in tolerance
            }

            // Proportional control for turn power
            double power = KpTurn * error;

            // Only heading correction, no X/Y drive
            setMotorPowers(0, 0, power);

            telemetry.addLine("Turning");
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error",error);
            telemetry.addData("Turn Power", power);
            telemetry.update();
        }
        stopMotors();
    }

    private double normalizeAngle(double angle){
        return Math.IEEEremainder(angle, 360.0);
    }

    /// Reset all errors and timers for pathing methods
    private void resetPidTimers(){
        pidTimerX.reset();
        pidTimerY.reset();
        lastXError = Double.NaN;
        lastYError = Double.NaN;
    }

    /// Calculates drive power for the pathing methods using PID control
    private double pidDrivePower(double error, boolean isX) {
        double kp = isX ? KpDrive : KpDrive + 0.006;

        // Get the time delta for derivative calculation
        double deltaTime;
        if(isX) {
            deltaTime = pidTimerX.seconds();
            pidTimerX.reset();
        }else{
            deltaTime = pidTimerY.seconds();
            pidTimerY.reset();
        }

        final double MIN_DT = 1e-2;
        if(deltaTime < MIN_DT) deltaTime = MIN_DT;

        // Calculate derivative (rate of change of error)
        double lastError = isX ? lastXError : lastYError;
        double derivative;
        if (Double.isNaN(lastError)) {
            derivative = 0;
        } else {
            derivative = (error - lastError) / deltaTime;
        }

        // Update last error for next iteration
        if (isX) {
            lastXError = error;
        } else {
            lastYError = error;
        }

        // Calculate PID output
        double proportional = kp * error;
        double derivativeTerm = KdDrive * derivative;

        return proportional + derivativeTerm;
    }


    /// Sets the motor powers according to the calculated powers for pathing methods
    /// @param xPower the scaled x power
    /// @param yPower the scaled y power
    /// @param headingCorrection the scaled heading correction(it will not limit for you!)
    private void setMotorPowers(double xPower, double yPower, double headingCorrection) {
        // Compensate for robot heading (field-centric control)
        double headingRad = Math.toRadians(getAngle());

        // Apply field-centric transformation
        double forwardPower =  xPower * Math.cos(headingRad) + yPower * Math.sin(headingRad);
        double strafePower  = -xPower * Math.sin(headingRad) + yPower * Math.cos(headingRad);

        // Calculate mecanum motor powers
        // Forward = all motors same direction, Strafe = diagonal pattern
        double flPower = forwardPower + strafePower + headingCorrection;
        double frPower = forwardPower - strafePower - headingCorrection;
        double blPower = forwardPower - strafePower + headingCorrection;
        double brPower = forwardPower + strafePower - headingCorrection;

        // Find the largest motor power magnitude
        double maxMotorPower = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );

        // Only scale if any motor power exceeds the limit
        if (maxMotorPower > AUTO_MAX_POWER) {
            double scaleFactor = AUTO_MAX_POWER / maxMotorPower;
            fl.setPower(flPower * scaleFactor);
            fr.setPower(frPower * scaleFactor);
            bl.setPower(blPower * scaleFactor);
            br.setPower(brPower * scaleFactor);
        } else {
            // No scaling needed, powers are already within limits
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }



    /// Sends any telemetry
    private void sendTelemetry(String label, double xErr, double yErr, double xPow, double yPow, double headingCorr) {
        telemetry.addLine("Following a " + label);
        telemetry.addData("X Coord", getX());
        telemetry.addData("Y Coord", getY());
        telemetry.addData("Heading", getAngle());
        telemetry.addData("X Error", xErr);
        telemetry.addData("Y Error", yErr);
        telemetry.addData("X Power", xPow);
        telemetry.addData("Y Power", yPow);
        telemetry.addData("Heading Corr", headingCorr);
        telemetry.update();
    }


    /// Sets all powers to 0
    private void stopMotors() {
        for (DcMotorEnhanced motor : new DcMotorEnhanced[]{fl, fr, bl, br}) {
            motor.setPower(0);
        }
    }

    /** Checks if the robot is not moving. Takes in target coords to check distance again.
     * Just checks if the distance to target is not updating
     * @param targetX the x coordinate you want to go to
     * @param targetY the y coordinate you want to go to
    */
    private boolean notStuck(double targetX, double targetY) {
        //Apply the pythagorean theorem for total distance moved in both directions
        double currentDistance = Math.sqrt(Math.pow(targetX - getX(), 2) + Math.pow(targetY - getY(), 2));

        if (Math.abs(currentDistance - lastDistanceToTarget) > 5) {
            lastDistanceToTarget = currentDistance;
            stuckTimer.reset();
            return true;
        }

        return stuckTimer.milliseconds() < PATH_TIMEOUT_MS;
    }


    /// Waits until the supplied condition is true
    /// @param condition the supplier condition to wait for. Pass in with () -> myCondition
    protected void waitUntil(@NonNull Supplier<Boolean> condition) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!condition.get() && opModeIsActive()) {
            // 0 is a way to get it to wait for the minimum 50ms
            wait(0);
        }
    }

    ///  Waits until the supplied condition is true, with a timeout option
    /// @param condition the supplier condition to wait for. Pass in with () -> myCondition
    protected void waitUntil(@NonNull Supplier<Boolean> condition, long timeoutMs) {
        if (timeoutMs <= 0) return;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!condition.get() && opModeIsActive() && timer.milliseconds() < timeoutMs) {
            // 0 is a way to get it to wait for the minimum 50ms
            wait(0);
        }
    }
}
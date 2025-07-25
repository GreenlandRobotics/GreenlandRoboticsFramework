package gcsrobotics.framework.hardware;

import static gcsrobotics.framework.Constants.ENCODER_TOLERANCE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import gcsrobotics.framework.OpModeBase;
@SuppressWarnings("unused")
public class DcMotorEnhanced {
    private final DcMotor motor;
    private double DEFAULT_SPEED = 1;

    public DcMotorEnhanced(DcMotor motor) {
        this.motor = motor;
    }

    public DcMotorEnhanced(DcMotor motor, double DEFAULT_SPEED){
        this.motor = motor;
        this.DEFAULT_SPEED = DEFAULT_SPEED;
    }

    public void setPosAndWait(int targetPosition, OpModeBase opmode){
        setPosAndWait(targetPosition,DEFAULT_SPEED,opmode);
    }
    public void setPosAndWait(int targetPosition, double speed,OpModeBase opmode){
        setPosition(targetPosition,speed);
        while(!isAtTarget()){
            opmode.sleep(10);
        }
    }

    /// Sets the given motor to go to a certain position, at full speed.
    /// If you want to vary the speed, add another parameter with the speed you want
    public void setPosition(int targetPosition){
        this.setPosition(targetPosition,DEFAULT_SPEED);
    }
    /// Sets the given motor to go to a certain position at a given speed
    public void setPosition(int targetPosition,double speed){
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }

    public void setDefaultSpeed(double DEFAULT_SPEED){
        this.DEFAULT_SPEED = DEFAULT_SPEED;
    }

    public double getDefaultSpeed(){return DEFAULT_SPEED;}

    public void reset(){
        DcMotor.RunMode temp = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(temp);
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - motor.getTargetPosition()) <= ENCODER_TOLERANCE;
    }

    // === Forward motor control with minimal effort ===
    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection(){
        return motor.getDirection();
    }

    /// --- Here you can still access motor directly ---
    public DcMotor getBaseMotor() {
        return motor;
    }
}

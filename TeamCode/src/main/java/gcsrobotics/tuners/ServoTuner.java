package gcsrobotics.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import gcsrobotics.framework.OpModeBase;
import gcsrobotics.framework.hardware.Claw;

///This class reads out the current position of a servo and allows you to adjust it,
/// meaning it is very useful for finding servo limits.
/// You can set these limits with Servo.scaleRange(double min, double max).
/// You can also set those limits with Claw.setLimits(double min, double max).
/// @see Servo#scaleRange(double min, double max)
/// @see Claw#setLimits(double min,double max)
@TeleOp(name="Servo Tuner")
@Config
//@Disabled
public class ServoTuner extends OpModeBase {//Replace this with the name that you want to tune

    public static double servoPos = 0.5;
    protected void runInit(){
        telemetry.addData("Status","Ready");
        telemetry.update();
    }

    protected void run(){
        while(opModeIsActive()) {

            telemetry.addData("Servo Position",servoPos);
            telemetry.update();

            //Replace 'servo' with the servo you want to tune
            servo.setPosition(servoPos);

        }
    }


}

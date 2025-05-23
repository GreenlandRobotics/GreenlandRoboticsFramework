package gcsrobotics.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class OpModeBase extends LinearOpMode {
    public DcMotorEnhanced fl,fr,bl,br,arm;
    public Servo claw;
    public GoBildaPinpointDriver odo;


    public abstract void runInit();
    public abstract void run();


    @Override
    public void runOpMode(){
        fl = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"fl"));
        fr = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"fr"));
        bl = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"bl"));
        br = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"br"));
        arm = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"arm"));

        claw = hardwareMap.get(Servo.class,"claw");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.resetPosAndIMU();

        // Set motor directions, change the ones that need to be reversed
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        runInit();
        waitForStart();
        run();

    }
}

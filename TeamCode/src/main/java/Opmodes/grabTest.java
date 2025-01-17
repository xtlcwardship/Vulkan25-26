package Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
@Autonomous(name = "GrabTest", group = "Test1")
public class grabTest extends LinearOpMode {
    private Servo shoulderLeft1, shoulderRight1, wrist, turnGrabber, openGrabber;
    private DcMotor slideythingy1;
    private double mult = 310.800311;
    private DistanceSensor detect;
    @Override
    public void runOpMode() throws InterruptedException {

        shoulderLeft1 = hardwareMap.get(Servo.class, "shit2");
        shoulderLeft1.setPosition(0.05);
        shoulderRight1 = hardwareMap.get(Servo.class, "shit1");
        shoulderRight1.setPosition(0.95);
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        openGrabber.setPosition(0.1);
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0.52);
        turnGrabber = hardwareMap.get(Servo.class, "turnGrabber");
        //turnGrabber.setPosition(0.6);
        detect = hardwareMap.get(DistanceSensor.class, "detect");
        slideythingy1 = hardwareMap.get(DcMotor.class, "slide1");
        slideythingy1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //IF PROBLEM CHANGE TO RUN_TO_POSITION
        slideythingy1.setDirection(DcMotor.Direction.REVERSE);
        slideythingy1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        shoulderLeft1.setPosition(0.0515);
        shoulderRight1.setPosition(0.9485);
        wrist.setPosition(1);
        //turnGrabber.setPosition(0.8);
        openGrabber.setPosition(0.5);
        sleep(500);
        if (slideythingy1.getCurrentPosition() < mult*(detect.getDistance(DistanceUnit.INCH)))
        {
            while (slideythingy1.getCurrentPosition() < mult*(detect.getDistance(DistanceUnit.INCH)))
            {
                slideythingy1.setPower(1);
            }

            slideythingy1.setPower(0);

        }
        //sleep(500);
        //turnGrabber.setPosition(0.8);
        openGrabber.setPosition(0.1);
        sleep(200);
        shoulderLeft1.setPosition(0.175);
        shoulderRight1.setPosition(0.825);
        //turnGrabber.setPosition(0.8);
        //telemetry.addData("Distance (cm): ", detect.getDistance(DistanceUnit.INCH));

    }
    }


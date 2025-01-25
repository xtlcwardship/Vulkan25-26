package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class testy extends LinearOpMode {
    //private Servo one;
    //private CRServo two;
    private Servo three;
    private Servo four;
    //private Servo five;
    //private Servo six;
    @Override
    public void runOpMode() throws InterruptedException {
        //one = hardwareMap.get(Servo.class, "turnSlurp");
        //two = hardwareMap.get(CRServo.class, "slurp");
        three = hardwareMap.get(Servo.class, "leftShoulder");
        four = hardwareMap.get(Servo.class, "rightShoulder");
        //five = hardwareMap.get(Servo.class, "turnGrabber");
        //six = hardwareMap.get(Servo.class, "openGrabber");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.y) {
                // move to 0 degrees.
                //one.setPosition(0.5);
                three.setPosition(0.6);
                four.setPosition(0.4);
                //five.setPosition(0);
                //six.setPosition(0.05);
            }
            else if (gamepad1.x) {
                // move to 90 degrees.
                //one.setPosition(0.88);
                three.setPosition(0.25);
                four.setPosition(0.75);
                //five.setPosition(0.75);
                //six.setPosition(0.23);
            }

            else if (gamepad1.a) {
                // move to 90 degrees.
                //one.setPosition(0.65);
                three.setPosition(0.55);
                four.setPosition(0.45);
                //five.setPosition(0.4);
                //six.setPosition(0.23);
            }
            //telemetry.addData("Servo Position", one.getPosition());
            telemetry.addData("Servo Position", three.getPosition());
            telemetry.addData("Servo Position", four.getPosition());
            //telemetry.addData("Servo Position", five.getPosition());
            //telemetry.addData("Servo Position", six.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

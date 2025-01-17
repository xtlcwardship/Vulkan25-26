package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testy extends LinearOpMode {
    private Servo one;
    private Servo two;
    private DcMotor slideythingy1;
    private Servo three;
    //private Servo four;
    //private Servo five;
    @Override
    public void runOpMode() throws InterruptedException {
        slideythingy1 = hardwareMap.get(DcMotor.class, "slide1");
        slideythingy1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //IF PROBLEM CHANGE TO RUN_TO_POSITION
        slideythingy1.setDirection(DcMotor.Direction.REVERSE);
        slideythingy1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        one = hardwareMap.get(Servo.class, "shit2");
        two = hardwareMap.get(Servo.class, "shit1");
        three = hardwareMap.get(Servo.class, "wrist");
        //four = hardwareMap.get(Servo.class, "die4");
        //five = hardwareMap.get(Servo.class, "die5");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double slidePower1 = 2.0;
            double stop = 0.0;

            if (gamepad1.y) {
                // move to 0 degrees.
                one.setPosition(0.07);
                two.setPosition(0.93);
                three.setPosition(0.75);
                //four.setPosition(0.0);
                //five.setPosition(0.0);
            }
            else if (gamepad1.x) {
                // move to 90 degrees.
                one.setPosition(0.2);
                two.setPosition(0.8);
                three.setPosition(0.75);
                //four.setPosition(0.5);
                //five.setPosition(0.5);
            }
            if (gamepad2.dpad_up && slideythingy1.getCurrentPosition() < 2456) {
                slideythingy1.setPower(slidePower1);
            } else if (gamepad2.dpad_down && slideythingy1.getCurrentPosition() > 0) {
                slideythingy1.setPower(-slidePower1);
            } else {
                slideythingy1.setPower(stop);
            }
            telemetry.addData("Servo Position", one.getPosition());
            telemetry.addData("Servo Position", two.getPosition());
            telemetry.addData("Servo Position", three.getPosition());
            //telemetry.addData("Servo Position", four.getPosition());
            //telemetry.addData("Servo Position", five.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

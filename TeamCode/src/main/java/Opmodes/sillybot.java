package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "sillybot", group = "Teleop")
public class sillybot extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor slideythingy1;
    private Servo openGrabber;
    private boolean b2Pressable;
    private boolean bLast;
    private Servo turnGrabber;
    private boolean y2Pressable;
    private boolean yLast;
    private Servo wrist;
    private boolean a2Pressable;
    private boolean aLast;
    private Servo shoulderLeft1, shoulderRight1;
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        slideythingy1 = hardwareMap.get(DcMotor.class, "slide1");
        slideythingy1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //IF PROBLEM CHANGE TO RUN_TO_POSITION
        slideythingy1.setDirection(DcMotor.Direction.REVERSE);
        slideythingy1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        openGrabber.setPosition(0.11);
        turnGrabber = hardwareMap.get(Servo.class, "turnGrabber");
        turnGrabber.setPosition(0.8);
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0.30);
        shoulderLeft1 = hardwareMap.get(Servo.class, "shit2");
        shoulderLeft1.setPosition(0.1);
        shoulderRight1 = hardwareMap.get(Servo.class, "shit1");
        shoulderRight1.setPosition(0.9);
        left1 = hardwareMap.get(DcMotor.class, "leftFront");
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2 = hardwareMap.get(DcMotor.class, "leftBack");
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1 = hardwareMap.get(DcMotor.class, "rightFront");
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2 = hardwareMap.get(DcMotor.class, "rightBack");
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b2Pressable = false;
        bLast = false;
        y2Pressable = false;
        a2Pressable = false;
        yLast = false;
        aLast = false;

        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double slidePower1 = 2.0;
            double stop = 0.0;
            /*double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            left1.setPower(leftPower);
            left2.setPower(leftPower);
            right1.setPower(rightPower);
            right2.setPower(rightPower);*/
            //DRIVE
            /*
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();
*/
            //SHOULDER
            if (gamepad1.y && !gamepad1.b && !gamepad1.a && !gamepad1.x && !gamepad1.right_bumper)
            {
                if (wrist.getPosition() == 1.0) {
                    wrist.setPosition(0.52);
                    sleep(300);
                    shoulderLeft1.setPosition(0.05);
                    shoulderRight1.setPosition(0.95);
                }
                else {
                    shoulderLeft1.setPosition(0.05);
                    shoulderRight1.setPosition(0.95);
                }
            }
            else if (gamepad1.b && !gamepad1.x && !gamepad1.a && !gamepad1.y && !gamepad1.right_bumper)
            {
                if (wrist.getPosition() == 1.0) {
                    wrist.setPosition(0.52);
                    sleep(300);
                    shoulderLeft1.setPosition(0.06);
                    shoulderRight1.setPosition(0.94);
                }
                else {
                    shoulderLeft1.setPosition(0.06);
                    shoulderRight1.setPosition(0.94);
                }
            }
            else if (gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y && !gamepad1.right_bumper)
            {
                shoulderLeft1.setPosition(0.3); //0.35
                shoulderRight1.setPosition(0.7);//0.65
                sleep(500);
                wrist.setPosition(1.0);

            }
            else if (gamepad1.x && !gamepad1.b && !gamepad1.a && !gamepad1.y && !gamepad1.right_bumper)
            {
                if (wrist.getPosition() == 1.0) {
                    wrist.setPosition(0.52);
                    sleep(300);
                    shoulderLeft1.setPosition(0.02);
                    shoulderRight1.setPosition(0.98);
                }
                else {
                    shoulderLeft1.setPosition(0.02);
                    shoulderRight1.setPosition(0.98);
                }
            }

            else if (gamepad1.right_bumper && !gamepad1.y && !gamepad1.b && !gamepad1.a && !gamepad1.x) {
                if (wrist.getPosition() == 1.0) {
                    wrist.setPosition(0.52);
                    sleep(300);
                    shoulderLeft1.setPosition(0.2);
                    shoulderRight1.setPosition(0.8);
                }
                else {
                    shoulderLeft1.setPosition(0.2);
                    shoulderRight1.setPosition(0.8);
                }
            }


            /*if (gamepad1.right_trigger < 0.15) {
                shoulderLeft1.setPosition(shoulderLeft1.getPosition() - 0.05 * gamepad1.right_trigger);
                shoulderRight1.setPosition(shoulderRight1.getPosition() + 0.05 * gamepad1.right_trigger);
            } else if (gamepad1.right_trigger > 0.15) {
                shoulderLeft1.setPosition(shoulderLeft1.getPosition() - 0.05 * gamepad1.right_trigger);
                shoulderRight1.setPosition(shoulderRight1.getPosition() + 0.05 * gamepad1.right_trigger);
            } */
            if (shoulderLeft1.getPosition() <= 0.25) {
                //SLIDE
                if (gamepad2.dpad_up && slideythingy1.getCurrentPosition() < 1861) {
                    slideythingy1.setPower(slidePower1);
                } else if (gamepad2.dpad_down && slideythingy1.getCurrentPosition() > 0) {
                    slideythingy1.setPower(-slidePower1);
                } else {
                    slideythingy1.setPower(stop);
                }

                if ((shoulderLeft1.getPosition() == 0.2 || shoulderLeft1.getPosition() == 0.06)) {
                    if (gamepad2.a && !aLast) {
                        a2Pressable = !a2Pressable;
                    }

                    if (a2Pressable) {
                        wrist.setPosition(0.8);
                    }
                    else {
                        wrist.setPosition(0.52);
                    }

                    aLast = gamepad2.a;
                }
            }

            else {
                //SLIDE
                if (gamepad2.dpad_up && slideythingy1.getCurrentPosition() < 2695) {
                    slideythingy1.setPower(slidePower1);
                } else if (gamepad2.dpad_down && slideythingy1.getCurrentPosition() > 0) {
                    slideythingy1.setPower(-slidePower1);
                } else {
                    slideythingy1.setPower(stop);
                }
            }
            //OPEN GRABBER
            if (gamepad2.b && !bLast) {
                b2Pressable = !b2Pressable;
            }

            if (b2Pressable) {
                openGrabber.setPosition(0.45);
            }
            else {
                openGrabber.setPosition(0.11);
            }

            bLast = gamepad2.b;

            //TURN GRABBER
            if (gamepad2.y && !yLast) {
                y2Pressable = !y2Pressable;
            }

            if (y2Pressable) {
                turnGrabber.setPosition(0.0);
            }
            else {
                turnGrabber.setPosition(0.8);
            }

            yLast = gamepad2.y;


            telemetry.addData("Slide", "extend");
            telemetry.addData("Slide", slideythingy1.getCurrentPosition());
            telemetry.addData("shoulder", " " + shoulderLeft1.getPosition());
            telemetry.addData("Grabber up/down at position:", " " + wrist.getPosition());
            telemetry.addData("Grabber turned at position:", " " + turnGrabber.getPosition());
            telemetry.addData("Grabber open at position:", " " + openGrabber.getPosition());
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();

    }
}


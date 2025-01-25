package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "alphabot", group = "AAA")
public class alphabot extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rSlide, lSlide;
    private DcMotor outSlide;
    private DcMotor left1, left2, right1, right2;
    private Servo turnSlurp;
    private boolean rbump2Pressable;
    private boolean rbumpLast;
    private CRServo slurp;
    private Servo openGrabber, turnGrabber;
    private boolean b2Pressable;
    private boolean bLast;
    private boolean a2Pressable;
    private boolean aLast;
    private boolean y2Pressable;
    private boolean yLast;
    private Servo leftShoulder, rightShoulder;
    //add boolean pressables + lasts

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //switch slides to RUN_TO_POSITION
        rSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlide.setDirection(DcMotor.Direction.FORWARD);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setTargetPosition(0);
        rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlide.setPower(1);
        lSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlide.setDirection(DcMotor.Direction.REVERSE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setTargetPosition(0);
        lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlide.setPower(1);
        outSlide = hardwareMap.get(DcMotor.class, "out");
        outSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outSlide.setDirection(DcMotor.Direction.REVERSE);
        outSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left1 = hardwareMap.get(DcMotor.class, "leftFront");
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2 = hardwareMap.get(DcMotor.class, "leftBack");
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1 = hardwareMap.get(DcMotor.class, "rightFront");
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2 = hardwareMap.get(DcMotor.class, "rightBack");
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");
        turnSlurp.setPosition(0.1);
        rbump2Pressable = false;
        rbumpLast = false;
        leftShoulder = hardwareMap.get(Servo.class, "leftShoulder");
        //leftShoulder.setPosition(0.55);
        rightShoulder = hardwareMap.get(Servo.class, "rightShoulder");
        //rightShoulder.setPosition(0.45);
        turnGrabber = hardwareMap.get(Servo.class, "turnGrabber");
        //turnGrabber.setPosition(0.75);
        a2Pressable = false;
        aLast = false;
        y2Pressable = false;
        yLast = false;
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        openGrabber.setPosition(0.05);
        b2Pressable = false;
        bLast = false;
        slurp = hardwareMap.get(CRServo.class, "slurp");
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double slidePower = 1;
            double slidePower1 = 0.5;
            double stop = 0.0;

            //DRIVE
            //COMMENT OUT WHEN TRANSFERRING TO OTHER PROJECT
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
            //

            //UPSLIDES
            if (gamepad1.dpad_up && rSlide.getCurrentPosition() < 950 && lSlide.getCurrentPosition() < 950) {
                rSlide.setTargetPosition(rSlide.getTargetPosition() + 1);
                lSlide.setTargetPosition(lSlide.getTargetPosition() + 1);
            } else if (gamepad1.dpad_down && rSlide.getCurrentPosition() > 15 && lSlide.getCurrentPosition() > 15) {
                rSlide.setTargetPosition(rSlide.getTargetPosition() - 1);
                lSlide.setTargetPosition(lSlide.getTargetPosition() - 1);
            }

            //OUTSLIDE
            if (gamepad2.dpad_up && outSlide.getCurrentPosition() < 750) {
                telemetry.addLine("happens");
                outSlide.setPower(slidePower1);
            } else if (gamepad2.dpad_down && outSlide.getCurrentPosition() > 0) {
                outSlide.setPower(-slidePower1);
            } else {
                outSlide.setPower(stop);
            }

            //SLURP
            //turn
            if (gamepad2.right_bumper && !rbumpLast) {
                rbump2Pressable = !rbump2Pressable;
            }

            if (rbump2Pressable) {
                turnSlurp.setPosition(0.88);
            }
            else {
                turnSlurp.setPosition(0.65);
            }

            rbumpLast = gamepad2.right_bumper;

            //slurp

            //GRABBER
            //open
            if (gamepad2.b && !bLast) {
                b2Pressable = !b2Pressable;
            }

            if (b2Pressable) {
                openGrabber.setPosition(0.23);
            }
            else {
                openGrabber.setPosition(0.05);
            }

            bLast = gamepad2.b;

            //turn
            /*
            if (gamepad2.a && !aLast) {
                a2Pressable = !a2Pressable;
            }

            if (a2Pressable) {
                turnGrabber.setPosition(0.1);
            }
            else {
                turnGrabber.setPosition(0.4);
            }


             */
            //aLast = gamepad2.a;
/*
            if (gamepad2.y && !yLast) {
                y2Pressable = !y2Pressable;
            }

            if (y2Pressable) {
                turnGrabber.setPosition(0.75);
            }
            else {
                turnGrabber.setPosition(0.4);
            }

            yLast = gamepad2.y;


 */
            //SHOULDER

            telemetry.addData("Slurp turned to: ", turnSlurp.getPosition());
            telemetry.addData("Left shoulder at: ", leftShoulder.getPosition());
            telemetry.addData("Right shoulder at: ", rightShoulder.getPosition());
            telemetry.addData("Grabber turned to: ", turnGrabber.getPosition());
            if (openGrabber.getPosition() < 0.1) {
                telemetry.addData("Grabber closed: ", openGrabber.getPosition());
            }
            else {
                telemetry.addData("Grabber open: ", openGrabber.getPosition());
            }
            telemetry.addData("Right Slide at: ", rSlide.getCurrentPosition());
            telemetry.addData("Left Slide at: ", lSlide.getCurrentPosition());
            telemetry.addData("Out Slide at: ", outSlide.getCurrentPosition());
            //COMMENT OUT WHEN TRANSFERRING TO OTHER PROJECT
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
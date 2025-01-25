package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class megaMacro extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rSlide, lSlide;
    private DcMotor outSlide;
    private DcMotor left1, left2, right1, right2;
    private Servo turnSlurp;
    private CRServo slurp;
    private Servo openGrabber, turnGrabber;
    private boolean y2Pressable;
    private boolean yLast;
    private Servo leftShoulder, rightShoulder;
    //add boolean pressables + lasts

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlide.setDirection(DcMotor.Direction.FORWARD);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlide.setDirection(DcMotor.Direction.REVERSE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        //turnSlurp.setPosition(0.5);
        leftShoulder = hardwareMap.get(Servo.class, "leftShoulder");
        //leftShoulder.setPosition(0.55);
        rightShoulder = hardwareMap.get(Servo.class, "rightShoulder");
        //rightShoulder.setPosition(0.45);
        turnGrabber = hardwareMap.get(Servo.class, "turnGrabber");
        //turnGrabber.setPosition(0.75);
        y2Pressable = false;
        yLast = false;
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        slurp = hardwareMap.get(CRServo.class, "slurp");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double slidePower = 0.5;
            double stop = 0.0;

            if (gamepad2.y) {
                openGrabber.setPosition(0.23);
                turnGrabber.setPosition(0.38);
                leftShoulder.setPosition(0.6);
                rightShoulder.setPosition(0.4);
                //turnSlurp.setPosition(0.67);
                sleep(500);
                openGrabber.setPosition(0.05);
            }
            yLast = gamepad2.y;
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

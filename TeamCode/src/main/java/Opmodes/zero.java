package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class zero extends LinearOpMode{
    private Servo one;
    private Servo two;
    private Servo three;
    private Servo four;
    private Servo five;
    private Servo six;

    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.get(Servo.class, "turnSlurp");
        //two = hardwareMap.get(Servo.class, "slurp");
        /*three = hardwareMap.get(Servo.class, "leftShoulder");
        four = hardwareMap.get(Servo.class, "rightShoulder");
        five = hardwareMap.get(Servo.class, "turnGrabber");
        six = hardwareMap.get(Servo.class, "openGrabber");

         */
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            one.setPosition(0.65);
            /*
            two.setPosition(0);
            three.setPosition(0);
            four.setPosition(0);
            five.setPosition(0);

             */
        }
        telemetry.addData("Servo Position", one.getPosition());
        telemetry.addData("Servo Position", two.getPosition());
        telemetry.addData("Servo Position", three.getPosition());
        telemetry.addData("Servo Position", four.getPosition());
        telemetry.addData("Servo Position", five.getPosition());
        telemetry.addData("Servo Position", six.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
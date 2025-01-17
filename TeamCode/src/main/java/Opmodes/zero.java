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

        @Override
        public void runOpMode() throws InterruptedException {
            //one = hardwareMap.get(Servo.class, "shit1");
            //two = hardwareMap.get(Servo.class, "shit2");
            //three = hardwareMap.get(Servo.class, "openGrabber");
            four = hardwareMap.get(Servo.class, "turnGrabber");
            //five = hardwareMap.get(Servo.class, "wrist");
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                four.setPosition(0);
                sleep(1000);
                four.setPosition(0.8);
/*
                if (gamepad1.y) {
                    // move to 0 degrees.
                    one.setPosition(0);
                    two.setPosition(1.0);
                    three.setPosition(0);
                    four.setPosition(0);
                    five.setPosition(0);
                }
                else if (gamepad1.x) {
                    // move to 90 degrees.
                    one.setPosition(0.5);
                    two.setPosition(0.5);
                    three.setPosition(0.5);
                    four.setPosition(0.5);
                    five.setPosition(0.5);
                }

 */
                telemetry.addData("Servo Position", one.getPosition());
                telemetry.addData("Servo Position", two.getPosition());
                telemetry.addData("Servo Position", three.getPosition());
                telemetry.addData("Servo Position", four.getPosition());
                telemetry.addData("Servo Position", five.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }

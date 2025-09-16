package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class learning extends LinearOpMode {

    Servo servo;

    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.start) {
                servo.setPosition(0.0);
            }
            if(gamepad1.a) {
                servo.setPosition(0.5);
            }
            if(gamepad1.dpad_up) {
                motor.setPower(1.0);
            }
            if(gamepad1.dpad_down) {
                motor.setPower(-1.0);
            }
            if(gamepad1.dpad_left) {
                motor.setPower(0.0);





            }

            }
        }


}

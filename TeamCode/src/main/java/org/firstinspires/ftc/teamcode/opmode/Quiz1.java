package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "1st Test ", group = "TeleOp")
public class Quiz1 extends LinearOpMode {

    private int john = 15;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private DcMotor IntakeMotor;

    private DcMotor ShooterMotor;
    private DcMotor ShooterMotor2;
    private DcMotor TransportMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        ShooterMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
        ShooterMotor2 = hardwareMap.get(DcMotor.class, "ShooterMotor2");
        TransportMotor = hardwareMap.get(DcMotor.class, "TransportMotor");

        // Reverse direction on right-side motors
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        TransportMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Set zero power behavior
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TransportMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (gamepad1.a) {
            IntakeMotor.setPower(1);
            }
            else if (gamepad1.b) {
            ShooterMotor.setPower(1);
            ShooterMotor2.setPower(1);
            }
            else if (gamepad1.x) {TransportMotor.setPower(1);
            }
            else {
            IntakeMotor.setPower(0);
            ShooterMotor.setPower(0);
            ShooterMotor2.setPower(0);
            TransportMotor.setPower(0);
            }



            // Motor power calculations
            double frontLeftPower = speed + turn;
            double frontRightPower = speed - turn;
            double backLeftPower = speed + turn;
            double backRightPower = speed - turn;

            // Normalize values to keep within [-1, 1]
            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Apply power to motors
            double RightScale = 1;
            double LeftScale = 1;
            FrontLeft.setPower(frontLeftPower *= LeftScale);
            FrontRight.setPower(frontRightPower);
            BackLeft.setPower(backLeftPower );
            BackRight.setPower(backRightPower);

            // Telemetry output
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Intake Motor Power", IntakeMotor.getPower());
            telemetry.addData("Shooter Motor Power", ShooterMotor.getPower());
            telemetry.addData("Transfer Motor Power", TransportMotor.getPower());
            telemetry.addData("John", john);
            telemetry.update();
        }
    }
}

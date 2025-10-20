@TeleOp
package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class BigDrive extends LinearOpMode {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, IntakeMotor, ShooterMotor, ShooterMotor2, TransportMotor;

    double frontLeftPower = (0.15);
    double frontRightPower = (0.15);
    double backLeftPower = (0.15);
    double backRightPower = (0.15);

    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        // IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        // ShooterMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");
        // ShooterMotor2 = hardwareMap.get(DcMotor.class, "ShooterMotor2");
        // TransportMotor = hardwareMap.get(DcMotor.class, "TransportMotor");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()) {

               if (gamepad1.a) {
                   IntakeMotor.setPower(1.0);
               }
               else {
                   IntakeMotor.setPower(0);
               }
            //   else if (gamepad1.b) {
            //        ShooterMotor.setPower(1.0);
            //       ShooterMotor2.setPower(1.0);
            //   }
            //   else if (gamepad1.x) {TransportMotor.setPower(1);
            //   }//  else {
            //       IntakeMotor.setPower(0.0);
            //       ShooterMotor.setPower(0.0);
            //       ShooterMotor2.setPower(0.0);
            //        TransportMotor.setPower(0.0);
            //   }

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            FrontLeft.setPower(frontLeftPower);
            BackLeft.setPower(backLeftPower);
            FrontRight.setPower(frontRightPower);
            BackRight.setPower(backRightPower);

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            //  telemetry.addData("Intake Motor Power", IntakeMotor.getPower());
            // telemetry.addData("Shooter Motor Power", ShooterMotor.getPower());
            // telemetry.addData("Shooter Motor Power", ShooterMotor2.getPower());
            //  telemetry.addData("Transfer Motor Power", TransportMotor.getPower());
            telemetry.update();
        }

    }
}
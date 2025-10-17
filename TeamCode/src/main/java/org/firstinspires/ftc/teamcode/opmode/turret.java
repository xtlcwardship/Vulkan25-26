package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret", group = "TeleOp")
public class turret extends LinearOpMode {
    private static final double TARGETTX = 0.0;
    private static final double kP = 0.01;
    private static final double DEADZONE = 0.5;
    private static final double MIN_POSITION = 0.0; // Minimum servo position
    private static final double MAX_POSITION = 1.0; // Maximum servo position
    private static final double CENTER_POSITION = 0.5; // Center position

    // Shooter power constants based on target area
    private static final double MIN_SHOOTER_POWER = 0.2; // Minimum power when far away (small Ta)
    private static final double MAX_SHOOTER_POWER = 0.8; // Maximum power when close (large Ta)
    private static final double MIN_TARGET_AREA = 0.1;   // Smallest expected target area
    private static final double MAX_TARGET_AREA = 5.0;   // Largest expected target area
    private Servo rotationServo;
    private Limelight3A limelight;
    private DcMotor Shooter;
    private DcMotor Shooter2;
    private double currentPosition;

    @Override
    public void runOpMode() {
        // Map hardware
        rotationServo = hardwareMap.get(Servo.class, "rotationServo"); // Update name as needed
        rotationServo.setPosition(CENTER_POSITION); // Start at center position
        currentPosition = CENTER_POSITION;

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(9); // AprilTag pipeline index
        limelight.start();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            double shooterPower = MIN_SHOOTER_POWER;

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double error = TARGETTX - tx;

                // Calculate shooter power based on target area (distance)
                if (ta > 0) {
                    shooterPower = Range.clip(
                            MIN_SHOOTER_POWER + (ta - MIN_TARGET_AREA) *
                                    (MAX_SHOOTER_POWER - MIN_SHOOTER_POWER) / (MAX_TARGET_AREA - MIN_TARGET_AREA),
                            MIN_SHOOTER_POWER,
                            MAX_SHOOTER_POWER
                    );
                }

                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", ta);
                telemetry.addData("Error", error);
                telemetry.addData("ShooterPower", shooterPower);

                if (Math.abs(error) > DEADZONE) {
                    // Convert error to position adjustment
                    double positionAdjustment = kP * error;
                    currentPosition += positionAdjustment;

                    // Clamp position to valid range
                    currentPosition = Range.clip(currentPosition, MIN_POSITION, MAX_POSITION);

                    rotationServo.setPosition(currentPosition);
                    telemetry.addData("Status", "Tracking");
                } else {
                    telemetry.addData("Status", "Centered");
                }
            } else {
                telemetry.addData("Status", "No Data");
            }

            Shooter.setPower(shooterPower);
            Shooter2.setPower(shooterPower);

            telemetry.addData("ServoPosition", currentPosition);
            telemetry.update();

            sleep(20);
        }
    }
}
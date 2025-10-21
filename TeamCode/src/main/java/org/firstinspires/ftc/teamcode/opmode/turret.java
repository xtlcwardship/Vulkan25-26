package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret", group = "TeleOp")
public class turret extends LinearOpMode {
    private static final double TARGETTX = 0.0;

    // PID Constants - Tuned for motor control
    private static final double kP = 0.01;
    private static final double kI = 0.001;
    private static final double kD = 0.03;

    // PID State Variables
    private double previousError = 0.0;
    private double integral = 0.0;
    private double lastTime = 0.0;

    // Integral windup protection
    private static final double MAX_INTEGRAL = 5.0;

    // Control parameters
    private static final double DEADZONE = 0.3;
    private static final double MAX_MOTOR_POWER = 0.4; // Maximum motor power for rotation

    // Shooter power constants based on target area
    private static final double MIN_SHOOTER_POWER = 0.2;
    private static final double MAX_SHOOTER_POWER = 0.8;
    private static final double MIN_TARGET_AREA = 0.1;
    private static final double MAX_TARGET_AREA = 5.0;
    double frontLeftPower = (0.15);
    double frontRightPower = (0.15);
    double backLeftPower = (0.15);
    double backRightPower = (0.15);

    private DcMotor rotationMotor;
    private Limelight3A limelight;
    private DcMotor Shooter, Shooter2;
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, IntakeMotor, ShooterMotor, ShooterMotor2, TransportMotor;
    private Servo IntakeServo;

    private double calculatePID(double error) {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;

        // Avoid division by zero on first iteration
        if (dt <= 0) {
            dt = 0.02;
        }

        // Proportional term
        double proportional = kP * error;

        // Integral term with windup protection
        integral += error * dt;
        integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral_term = kI * integral;

        // Derivative term
        double derivative = (error - previousError) / dt;
        double derivative_term = kD * derivative;

        // Calculate PID output
        double output = proportional + integral_term + derivative_term;

        // Update state variables for next iteration
        previousError = error;
        lastTime = currentTime;

        return output;
    }

    @Override
    public void runOpMode() {
        // Map hardware
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize PID timing
        lastTime = getRuntime();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            double shooterPower = MIN_SHOOTER_POWER;
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

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double error = TARGETTX - tx;
                double absError = Math.abs(error);

                // Calculate shooter power based on target area (distance)
                if (ta > 0) {
                    shooterPower = Range.clip(
                            MAX_SHOOTER_POWER - (ta - MIN_TARGET_AREA) *
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

                // Simple control: if error is outside deadzone, move motor
                if (absError > DEADZONE) {
                    // Calculate PID output
                    double pidOutput = calculatePID(error);

                    // Convert PID output to motor power
                    double motorPower = Range.clip(pidOutput * 0.1, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

                    // Set motor power directly
                    rotationMotor.setPower(motorPower);

                    telemetry.addData("Status", "Tracking");
                    telemetry.addData("Motor Power", motorPower);
                    telemetry.addData("PID Output", pidOutput);
                } else {
                    // Stop motor when in deadzone
                    rotationMotor.setPower(0);
                    integral = 0.0; // Reset integral to prevent windup
                    telemetry.addData("Status", "Centered");
                }
            } else {
                // Stop motor when no target detected
                rotationMotor.setPower(0);
                integral = 0.0;
                telemetry.addData("Status", "No Data");
            }

            Shooter.setPower(shooterPower);
            Shooter2.setPower(shooterPower);

            telemetry.update();
            sleep(20);
        }
    }
}
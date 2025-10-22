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

    // PID Constants - Tuned to reduce wobble
    private static final double kP = 0.008;  // Reduced proportional gain
    private static final double kI = 0.0005; // Reduced integral gain
    private static final double kD = 0.08;   // Increased derivative gain for damping

    // PID State Variables
    private double previousError = 0.0;
    private double integral = 0.0;
    private double lastTime = 0.0;
    private double lastMotorPower = 0.0; // Track last motor power for smoothing

    // Integral windup protection
    private static final double MAX_INTEGRAL = 3.0; // Reduced maximum integral

    // Control parameters - Adjusted to reduce wobble
    private static final double DEADZONE = 0.5; // Increased deadzone
    private static final double MAX_MOTOR_POWER = 0.3; // Reduced max power
    private static final double VELOCITY_LIMIT = 0.15; // Maximum change in motor power per loop
    private static final double MIN_MOTOR_POWER = 0.05; // Minimum power to overcome friction

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
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, IntakeMotor;

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

        // Derivative term (damping)
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
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            FrontLeft.setPower(frontLeftPower);
            BackLeft.setPower(backLeftPower);
            FrontRight.setPower(frontRightPower);
            BackRight.setPower(backRightPower);
            if (gamepad1.a) {
                IntakeMotor.setPower(0.5);
            } else {
                IntakeMotor.setPower(0);
            }

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

                // Improved control with velocity limiting
                if (absError > DEADZONE) {
                    // Calculate PID output
                    double pidOutput = calculatePID(error);

                    // Convert PID output to motor power
                    double targetMotorPower = Range.clip(pidOutput * 0.08, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

                    // Apply velocity limiting to prevent rapid changes
                    double powerDifference = targetMotorPower - lastMotorPower;
                    double limitedDifference = Range.clip(powerDifference, -VELOCITY_LIMIT, VELOCITY_LIMIT);
                    double motorPower = lastMotorPower + limitedDifference;

                    // Apply minimum power threshold to overcome friction
                    if (Math.abs(motorPower) > 0 && Math.abs(motorPower) < MIN_MOTOR_POWER) {
                        motorPower = Math.signum(motorPower) * MIN_MOTOR_POWER;
                    }

                    rotationMotor.setPower(motorPower);
                    lastMotorPower = motorPower;

                    telemetry.addData("Status", "Tracking");
                    telemetry.addData("Motor Power", motorPower);
                    telemetry.addData("PID Output", pidOutput);
                } else {
                    // Gradually reduce motor power when in deadzone
                    double reducedPower = lastMotorPower * 0.8; // Gradual reduction
                    if (Math.abs(reducedPower) < 0.02) {
                        reducedPower = 0;
                    }

                    rotationMotor.setPower(reducedPower);
                    lastMotorPower = reducedPower;

                    // Reset integral when settled
                    if (Math.abs(reducedPower) < 0.01) {
                        integral = 0.0;
                    }

                    telemetry.addData("Status", "Centered");
                }
            } else {
                // Gradually stop motor when no target detected
                double reducedPower = lastMotorPower * 0.7;
                if (Math.abs(reducedPower) < 0.02) {
                    reducedPower = 0;
                }

                rotationMotor.setPower(reducedPower);
                lastMotorPower = reducedPower;
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
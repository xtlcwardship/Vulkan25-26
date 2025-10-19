
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

    // PID Constants - Tuned for smoother control
    private static final double kP = 0.005;  // Reduced proportional gain
    private static final double kI = 0.0005; // Reduced integral gain
    private static final double kD = 0.02;   // Reduced derivative gain

    // PID State Variables
    private double previousError = 0.0;
    private double integral = 0.0;
    private double lastTime = 0.0;

    // Integral windup protection
    private static final double MAX_INTEGRAL = 5.0; // Reduced maximum integral accumulation

    // Control parameters
    private static final double DEADZONE = 0.3; // Reduced deadzone for better precision
    private static final double SETTLING_DEADZONE = 0.15; // Even smaller zone for settling
    private static final long SETTLING_DELAY_MS = 100; // 100ms delay when settled
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

    // Settling control variables
    private long lastSettledTime = 0;
    private boolean isSettled = false;

    private double calculatePID(double error) {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;

        // Avoid division by zero on first iteration
        if (dt <= 0) {
            dt = 0.02; // Assume 20ms loop time
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

        // Apply velocity limiting to prevent large jumps
        double maxVelocity = 0.02; // Maximum position change per loop
        output = Range.clip(output, -maxVelocity, maxVelocity);

        // Update state variables for next iteration
        previousError = error;
        lastTime = currentTime;

        return output;
    }

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
        limelight.pipelineSwitch(0); // AprilTag pipeline index
        limelight.start();

        // Initialize PID timing
        lastTime = getRuntime();

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

                // Check if we're in the settling zone
                boolean inSettlingZone = absError <= SETTLING_DEADZONE;

                // Check if we're in the deadzone
                boolean inDeadzone = absError <= DEADZONE;

                // Update settling status
                if (inSettlingZone) {
                    if (!isSettled) {
                        lastSettledTime = System.currentTimeMillis();
                        isSettled = true;
                    }
                } else {
                    isSettled = false;
                }

                // Only make corrections if we're not settled or if enough time has passed
                long timeSinceSettled = System.currentTimeMillis() - lastSettledTime;
                boolean shouldMakeCorrection = !inSettlingZone || timeSinceSettled > SETTLING_DELAY_MS;

                if (absError > DEADZONE && shouldMakeCorrection) {
                    // Calculate PID output
                    double pidOutput = calculatePID(error);

                    // Convert PID output to position adjustment
                    // Scale PID output appropriately for servo range
                    double positionAdjustment = pidOutput * 0.05; // Reduced scale factor
                    currentPosition += positionAdjustment;

                    // Clamp position to valid range
                    currentPosition = Range.clip(currentPosition, MIN_POSITION, MAX_POSITION);

                    rotationServo.setPosition(currentPosition);
                    telemetry.addData("Status", "Tracking");

                    // PID debugging telemetry
                    telemetry.addData("PID Output", pidOutput);
                    telemetry.addData("Integral", integral);
                    telemetry.addData("Derivative", (error - previousError) / 0.02);
                } else if (inSettlingZone && isSettled) {
                    // Reset integral when settled to prevent windup
                    integral = 0.0;
                    telemetry.addData("Status", "Settled");
                    telemetry.addData("Settled Time", timeSinceSettled + "ms");
                } else if (inDeadzone) {
                    // Reset integral when in deadzone
                    integral = 0.0;
                    telemetry.addData("Status", "Centered");
                } else {
                    telemetry.addData("Status", "Waiting");
                }
            } else {
                // Reset integral when no target detected
                integral = 0.0;
                isSettled = false;
                telemetry.addData("Status", "No Data");
            }

            Shooter.setPower(shooterPower);
            Shooter2.setPower(shooterPower);

            telemetry.addData("ServoPosition", currentPosition);
            telemetry.addData("IsSettled", isSettled);
            telemetry.update();

            sleep(20);
        }
    }
}

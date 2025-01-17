package Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "Autonomous: Color Detection", group = "Autonomous")
public class LimelightAuton extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Start with pipeline 0
        limelight.start(); // Begin data polling

        telemetry.addData("Status", "Initialized, Waiting for Start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the latest result from Limelight
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    // Get color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    boolean redDetected = false;
                    boolean blueDetected = false;
                    //telemetry.addData("tx", result.getTx());
                    //telemetry.addData("ty", result.getTy());
                    //telemetry.addData("color result", result.getColorResults());

                    for (LLResultTypes.ColorResult colorResult : colorResults) {
                        double colorX = colorResult.getTargetXDegrees();
                        double colorY = colorResult.getTargetYDegrees();

                        // Sample logic for red and blue detection based on position or other parameters
                        if (colorX > colorY) {
                            redDetected = true;
                            telemetry.addData("Detected Color", "red");
                            // Add action for red detection
                            // e.g., move robot or activate mechanism
                        } else if (colorY > colorX) {
                            blueDetected = true;
                            telemetry.addData("Detected Color", "Blue");
                            // Add action for blue detection
                            // e.g., move robot or activate mechanism
                        }
                    }

                    if (!redDetected && !blueDetected) {
                        telemetry.addData("Color", "No target color detected");
                    }
                } else {
                    telemetry.addData("Limelight", "No valid data");
                }

                telemetry.update();
            }
        }
        limelight.stop();
    }

   /* // Helper methods to determine if the detected color matches red or blue based on position/criteria
    private boolean isRed(double x, double y) {
        // Custom logic to define what constitutes "red" target - adjust based on your criteria
        return x > 160 && x < 180 && y > -10 && y < 10; // Example values
    }

    private boolean isBlue(double x, double y) {
        // Custom logic to define what constitutes "blue" target - adjust based on your criteria
        return x < 125 && x > 100 && y < 10 && y > -10; // Example values
    }*/
}

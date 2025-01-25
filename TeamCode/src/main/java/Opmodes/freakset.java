package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "freakset")
public class freakset extends LinearOpMode {
    private DcMotor rSlide;
    private DcMotor lSlide;
    private DcMotor out;
    public void runOpMode() throws InterruptedException {
        rSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out = hardwareMap.get(DcMotor.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

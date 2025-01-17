package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "freakset")
public class freakset extends LinearOpMode {
    private DcMotor slideythingy;
    public void runOpMode() throws InterruptedException {
        slideythingy = hardwareMap.get(DcMotor.class, "slide1");
        slideythingy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

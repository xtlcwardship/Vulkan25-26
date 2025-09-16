package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */


@Autonomous(name = "trialAuto")
public class theMoreYouKnow extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    //public Servo ect;
    //public DcMotor ect2;

    // Robot Positions
    private final Pose startPose = new Pose(0,0,Math.toRadians(0));




    //private Pathchain AllthePaths;

    public void buildPaths() {
//        examples
//                sampleDrop3 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(pickUp3),
//                        new Point(drop3)
//                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//        speciminGrab1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(drop3), new Point(wallPickPrep)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(new BezierLine(new Point(wallPickPrep), new Point(wallPick)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {

        }
    }



    @Override
    public void init() {
        //servos

        //motors

        //slides

        //other stuff

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        follower.followPath(total);
        setPathState(1);
    }

    @Override
    public void stop() {}
}

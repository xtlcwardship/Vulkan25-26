package Opmodes;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class BucketAuton extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload, park;
    private PathChain slurp1, slurp2, slurp3, score1, score2, score3;
    private final Pose startPose = new Pose(134, 45, Math.toRadians(-180));
    private final Pose slurp1Pose = new Pose(110, 21, Math.toRadians(-180));
    private final Pose score1Pose = new Pose(127, 14, Math.toRadians(-220));
    private final Pose slurp2Pose = new Pose(111, 11, Math.toRadians(-180));
    private final Pose score2Pose = new Pose(127, 14, Math.toRadians(-220));
    private final Pose slurp3Pose = new Pose(112, 4, Math.toRadians(-180));
    private final Pose score3Pose = new Pose(127,14, Math.toRadians(-220));
    //private final Pose grabPose = new Pose(133, 120, Math.toRadians(180));
    //private final Pose clipPose2 = new Pose(100, 76, Math.toRadians(-180));
    //private final Pose clipPose3 = new Pose(100, 74, Math.toRadians(-180));
   //private final Pose clipPose4 = new Pose(100, 72, Math.toRadians(-180));
    private final Pose end = new Pose(83, 43, Math.toRadians(-180));
    private Servo Lshoulder, Rshoulder, turnGrabber, openGrabber, slurp, turnSlurp;
    private DcMotor Lslide, Rslide, out;

    public void buildPaths()
    {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(score1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        slurp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(slurp1Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), slurp1Pose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(slurp1Pose.getHeading(), score1Pose.getHeading())
                .build();
        slurp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(slurp2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), slurp2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp2Pose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(slurp2Pose.getHeading(), score2Pose.getHeading())
                .build();
        slurp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(slurp3Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), slurp3Pose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slurp3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(slurp3Pose.getHeading(), score3Pose.getHeading())
                .build();
        park = new Path(new BezierLine(new Point(score3Pose), new Point(end)));
        park.setLinearHeadingInterpolation(score3Pose.getHeading(), end.getHeading());

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(slurp3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy())
                {
                    follower.followPath(park, true);
                    setPathState(8);
                }
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Rshoulder = hardwareMap.get(Servo.class, "rightShoulder");
        turnGrabber = hardwareMap.get(Servo.class, "turnGrabber");
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        Lslide = hardwareMap.get(DcMotor.class, "leftSlide");
        Rslide = hardwareMap.get(DcMotor.class, "rightSlide");
        turnSlurp = hardwareMap.get(Servo.class, "turnSlurp");
        Lslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //IF PROBLEM CHANGE TO RUN_TO_POSITION
        Lslide.setDirection(DcMotor.Direction.REVERSE);
        Lslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //IF PROBLEM CHANGE TO RUN_TO_POSITION
        Rslide.setDirection(DcMotor.Direction.FORWARD);
        Rslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();

    }
}

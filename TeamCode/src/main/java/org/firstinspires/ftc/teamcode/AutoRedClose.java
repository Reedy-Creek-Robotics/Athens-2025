package org.firstinspires.ftc.teamcode;

import android.icu.lang.UProperty;

import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class AutoRedClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(122,125,Math.toRadians(36.25));
    private final Pose scorePose = new Pose(84,84,Math.toRadians(47.8));
    private final Pose endPose = new Pose(127,84,0);
    private final Pose targetPose = new Pose(133,138);
    private Path intakeBalls;
    private PathChain preShoot;
    private DcMotorEx intakeMotor, outtakeMotor;

    @Override
    public void init() {
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setVelocityPIDFCoefficients(384, 3.37, 185, 2.914);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    
    @Override
    public void loop() {
        follower.update();
        pathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    
    @Override
    public void stop() {}

    public void buildPaths() {
        preShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(),0.8)
                .build();

        intakeBalls = new Path(new BezierLine(scorePose, endPose));
        intakeBalls.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), 0.25);
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preShoot,true);
                setPathState(1);
                outtakeMotor.setVelocity(175, AngleUnit.DEGREES);
                actionTimer.resetTimer();
                break;
            case 1:
                double currActTime = actionTimer.getElapsedTimeSeconds();
                if (currActTime > 10) {
                    follower.followPath(intakeBalls);
                    outtakeMotor.setVelocity(-90,AngleUnit.DEGREES);
                    intakeMotor.setPower(0.65);
                    setPathState(2);
                    actionTimer.resetTimer();
                }
                else if (currActTime > 5) {
                    intakeMotor.setPower(0.35);
                }
                break;
            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 8) {
                    setPathState(-1);
                    outtakeMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}


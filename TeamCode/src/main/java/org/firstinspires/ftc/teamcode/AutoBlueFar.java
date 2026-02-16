package org.firstinspires.ftc.teamcode;

import android.icu.lang.UProperty;

import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
public class AutoBlueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(60,9,Math.toRadians(90));
    private final Pose scorePose = new Pose(60,20,Math.toRadians(113.3));
    private final Pose endPose = new Pose(39,12,Math.toRadians(90));
    private final Pose targetPose = new Pose(11, 138);
    private PathChain intakeBalls;
    private PathChain preShoot;
    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotor;

    @Override
    public void init() {
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients coefficients = outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        coefficients.p = 67;

        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

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

        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void pathUpdate() {
        double currActTime = actionTimer.getElapsedTimeSeconds();
        switch (pathState) {
            case 0:
                follower.followPath(preShoot,true);
                setPathState(1);
                outtakeMotor.setVelocity(1615);
                actionTimer.resetTimer();
                break;
            case 1:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (currActTime > 7) {
                    follower.followPath(intakeBalls);
                    outtakeMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                    setPathState(2);
                    actionTimer.resetTimer();
                }
                else if (currActTime > 4) {
                    intakeMotor.setPower(0.5);
                }
                break;
            case 2:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (actionTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(-1);
                    follower.breakFollowing();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}


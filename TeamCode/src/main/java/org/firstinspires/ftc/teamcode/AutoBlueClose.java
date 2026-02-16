package org.firstinspires.ftc.teamcode;

import android.icu.lang.UProperty;

import com.pedropathing.geometry.BezierCurve;
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

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoBlueClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(22,125,Math.toRadians(143.75));
    private final Pose scorePose = new Pose(60,84,Math.toRadians(135));
    private final Pose spike1Pose = new Pose(20,84,Math.toRadians(180));
    private final Pose spike2PrePose = new Pose(54,60,Math.toRadians(180));
    private final Pose spike2Pose = new Pose(20,60,Math.toRadians(180));
    private final Pose targetPose = new Pose(11,138);
    private PathChain spike1Intake, preLoadShoot, spike1Shoot, spike2Intake;
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
        preLoadShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(),0.8)
                .build();

        spike1Intake = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spike1Pose))
                .setConstantHeadingInterpolation(spike1Pose.getHeading())
                .build();
        spike1Shoot = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pose, scorePose))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), scorePose.getHeading(), 0.8)
                .build();

        List<Pose> spike2IntakePoses = new ArrayList<Pose>();

        spike2IntakePoses.add(scorePose);
        spike2IntakePoses.add(spike2PrePose);
        spike2IntakePoses.add(spike2Pose);

        spike2Intake = follower.pathBuilder()
                .addPath(new BezierCurve(spike2IntakePoses))
                .setConstantHeadingInterpolation(spike2Pose.getHeading())
                .build();
    }

    public void pathUpdate() {
        double currActTime = actionTimer.getElapsedTimeSeconds();
        switch (pathState) {
            case 0:
                follower.followPath(preLoadShoot,true);
                setPathState(1);
                outtakeMotor.setVelocity(1275);
                actionTimer.resetTimer();
                break;
            case 1:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (currActTime > 6) {
                    follower.followPath(spike1Intake, true);
                    outtakeMotor.setVelocity(-600);
                    intakeMotor.setPower(0.6);
                    setPathState(2);
                    actionTimer.resetTimer();
                }
                else if (currActTime > 3.5) {
                    intakeMotor.setPower(0.5);
                }
                break;
            case 2:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (actionTimer.getElapsedTimeSeconds() > 5.2) {
                    setPathState(3);
                    follower.followPath(spike1Shoot, true);
                    outtakeMotor.setVelocity(1275);
                    intakeMotor.setPower(0);
                    actionTimer.resetTimer();
                }
                else if (currActTime > 3.5) {
                    intakeMotor.setPower(-0.15);
                    outtakeMotor.setVelocity(-2260);
                }
                break;
            case 3:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (currActTime > 7.5) {
                    follower.followPath(spike2Intake, true);
                    outtakeMotor.setVelocity(-600);
                    intakeMotor.setPower(0.6);
                    setPathState(4);
                    actionTimer.resetTimer();
                }
                else if (currActTime > 5) {
                    intakeMotor.setPower(0.5);
                }
                break;
            case 4:
                currActTime = actionTimer.getElapsedTimeSeconds();
                if (actionTimer.getElapsedTimeSeconds() > 5.2) {
                    setPathState(-1);
                    outtakeMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                    follower.breakFollowing();
                }
                else if (currActTime > 3.5
                ) {
                    intakeMotor.setPower(-0.15);
                    outtakeMotor.setVelocity(-2260);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}


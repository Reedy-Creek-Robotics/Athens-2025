package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

@Autonomous
public class AutoRedFar extends LinearOpMode {

    // Initialize program-wide elapsed time
    private static final ElapsedTime e = new ElapsedTime();

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        if (isStopRequested()) return;

        //122,125,36.25

        //131.5,137.5
    }

    // A homemade sleep method because the regular one is fucked
    public void sleepy(double time) {
        e.reset();
        while (e.seconds() < time && opModeIsActive());
    }
}

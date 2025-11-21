package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;


@Autonomous
public class AutoDrive extends LinearOpMode {

    // Initialize program-wide elapsed time
    private static final ElapsedTime e = new ElapsedTime();

    // Declare program-wide IMU
    private static IMU imu;

    // Initialize drivetrain motor array
    private static DcMotorEx[] driveTrain = new DcMotorEx[4];

    @Override
    public void runOpMode() throws InterruptedException {

        // Retrieve and initialize drivetrain motors from hardware map
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr");

        // Configure drivetrain motors to correct orientation, take note of directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        // Assign all drivetrain motors to drivetrain array, take note of indices of each motor
        driveTrain[0] = lf;
        driveTrain[1] = lr;
        driveTrain[2] = rf;
        driveTrain[3] = rr;

        // Configure drivetrain motors to resist motion when zero power
        for (int i = 0; i < 4; i++) driveTrain[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve and initialize IMU from hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Set IMU orientation based on Rev Control Hub orientation on bot
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        waitForStart();

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        if (isStopRequested()) return;


    }

    // A homemade sleep method because the regular one is fucked
    public void sleepy(double time) {
        e.reset();
        while (e.seconds() < time && opModeIsActive());
    }
}

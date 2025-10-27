package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
public class AutoDrive extends LinearOpMode {

    // Initialize program-wide elapsed time
    private static final ElapsedTime e = new ElapsedTime();

    // Speed of all drivetrain movement
    private final static double SPEED = 0.5;

    // Measured maximum angular velocity and acceleration
    private final static double maxAngularVelo = 0, maxAngularAccel = 0;


    // Declare program-wide IMU
    private static IMU imu;

    // Initialize drivetrain motor array
    private static DcMotorEx[] driveTrain = new DcMotorEx[4];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Autonomous ", "Started");
        telemetry.update();

        // Retrieve and initialize drivetrain motors from hardware map
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Configure drivetrain motors to correct orientation, take note of directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Assign all drivetrain motors to drivetrain array, take note of indices of each motor
        driveTrain[0] = frontLeftMotor;
        driveTrain[1] = backLeftMotor;
        driveTrain[2] = frontRightMotor;
        driveTrain[3] = backRightMotor;

        // Configure drivetrain motors to resist motion when zero power and reset encoders
        for (int i = 0; i < 4; i++) {
            driveTrain[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveTrain[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveTrain[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Retrieve and initialize IMU from hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Set IMU orientation based on Rev Control Hub orientation on bot
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) {}

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) driveTrain[i].setPower(1);

            e.reset();
            while (e.seconds() < 10 && opModeIsActive()) {

            }
            for (int i = 0; i < 4; i++) driveTrain[i].setPower(0);

            sleepy(10);
        }
    }

    // A homemade sleep method because the regular one is fucked
    public void sleepy(double time) {
        e.reset();
        while (e.seconds() < time && opModeIsActive()) {
        }
    }
}

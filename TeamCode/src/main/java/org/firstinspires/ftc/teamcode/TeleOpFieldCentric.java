package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Mecanum Field-Centric TeleOp", group="TeleOp")
public class TeleOpFieldCentric extends LinearOpMode {

    // Declare drivetrain motors
    private DcMotorEx lf, lr, rf, rr;

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize drivetrain motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        // Correct backward motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust orientation parameters to match control hub orientation on bot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();

        // OpMode loop
        while (opModeIsActive()) {
            // Take controller inputs
            double y = -gamepad1.left_stick_y; // Take left stick y-axis (forward/backward) reversed to correct
            double x = gamepad1.left_stick_x; // Take left stick x-axis (left/right)
            double rx = gamepad1.right_stick_x; // Take right stick x-axis (counter-clockwise/clockwise)

            // Reset default heading direction
            if (gamepad1.options) imu.resetYaw();

            // Take heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Counteract imperfect strafing
            rotX = rotX * 1.1;

            // Maintain motor power ratio between inputs
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            // calculate motor powers
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set motor powers to drivetrain
            lf.setPower(frontLeftPower);
            lr.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rr.setPower(backRightPower);
        }
    }
}
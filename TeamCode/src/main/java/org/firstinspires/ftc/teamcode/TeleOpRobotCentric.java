package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum Robot-Centric TeleOp", group="TeleOp")
public class TeleOpRobotCentric extends LinearOpMode {

    // Declare drivetrain motors
    private DcMotorEx lf, lr, rf, rr;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize drivetrain motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        // Correct backward motor directions
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // OpMode loop
        while (opModeIsActive()) {
            // Take controller inputs
            double y = gamepad1.left_stick_y; // Take left stick y-axis (forward/backward) reversed to correct bug
            double x = -gamepad1.left_stick_x; // Take left stick x-axis (left/right)
            double rx = -gamepad1.right_stick_x; // Take right stick x-axis (counter-clockwise/clockwise)

            // Maintain motor power ratio between inputs
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

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
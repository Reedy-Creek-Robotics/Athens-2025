package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Robot-Centric TeleOp", group="TeleOp")
public class TeleOpRobotCentric extends LinearOpMode {

    // Declare drivetrain motors
    private DcMotorEx lf, lr, rf, rr;
    private DcMotorEx intakeMotor;
    private CRServo sillyServo;

    private ElapsedTime t = new ElapsedTime();
    private boolean isToggled = false;

    private boolean previousGampad = false;

    private int state = 0;
    private int totalStates = 3; // e.g., Low, Medium, High

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize drivetrain motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        sillyServo = hardwareMap.get(CRServo.class, "sillyServo");

        // Correct backward motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // OpMode loop
        while (opModeIsActive()) {


            // Take controller inputs
            double y = -gamepad1.left_stick_y; // Take left stick y-axis (forward/backward) reversed to correct bug
            double x = gamepad1.left_stick_x; // Take left stick x-axis (left/right)
            double rx = gamepad1.right_stick_x; // Take right stick x-axis (counter-clockwise/clockwise)

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

            if (gamepad1.a && !previousGampad) {
                state = (state + 1) % totalStates; ; // Switch the state
            }

            if (state == 0)
            {
                sillyServo.setPower(1.0);
            }
            else if (state == 1)
            {
                sillyServo.setPower(-1.0);
            }
            else if (state == 2)
            {
                sillyServo.setPower(0.0);
            }

            previousGampad = gamepad1.a;
        }
    }
}
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

        long FI_lastToggleTime = 250; // milliseconds
        long FO_lastToggleTime = 250; // milliseconds
        int DEBOUNCE_DELAY = 250;

        int IntakePower = 0;

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

            if (gamepad1.dpad_down && (System.currentTimeMillis() - FI_lastToggleTime > DEBOUNCE_DELAY)) {
                IntakePower = IntakePower == -1? 0: -1; // Switch the state
                FI_lastToggleTime = System.currentTimeMillis();
            }

            if (gamepad1.dpad_up && (System.currentTimeMillis() - FO_lastToggleTime > DEBOUNCE_DELAY)) {
                IntakePower = IntakePower == 1? 0: 1;
                FO_lastToggleTime = System.currentTimeMillis();
            }

            intakeMotor.setPower(IntakePower);

        }
    }
}
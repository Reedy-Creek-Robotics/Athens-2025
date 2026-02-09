package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class TeleOpDrive extends LinearOpMode   {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static ElapsedTime e = new ElapsedTime();

    final private List<String> ready = Arrays.asList("Ready?", "Let's Go!", "準備完了!", "Look behind you.", "YIPPIE!!!", "It's Tiiime!", "In position", "Locked and loaded", "pwease pwess me >-<", "PLEASE NOTICE MEEE", "Ready to go!", "Yeehaw!");
    final double targetToTagDist = 15; // *Perpendicular* distance between a sensed AprilTag and the target point
    final double camToCenterDist = 9.7; // *Perpendicular* distance between the camera and the robot's center of rotation

    private static double[] centerToTargetVector = new double[2];

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare motors
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf"); // front right
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr"); // back right
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf"); // front left
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr"); // back left
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        // Variables for outtake finite state machine
        double outtakeMotorVelo = 0;
        double outtakeTimeMarker = 0; // time since last input

        // Variables for transfer finite state machine
        boolean transferOn = false; // transfer is transferring marker
        double transTimeMarker = 0; // time since last input

        // Variables for intake finite state machine
        boolean intoutOn = false; // intake is intaking
        boolean intinOn = false; // intake is outtaking
        double intakeTimeMarker = 0; // time since last input

        boolean fixItPlease = false;
        double fixItMarker = -0.2;

        boolean lockEndReady = false; // Bot locked onto target heading
        String currentFunny = "";

        // Reverse the right side motors. This may be wrong for setup.
        // If robot moves backwards when commanded to go forwards, reverse the left side instead.
        // See the note about this earlier on this page.

        // Correct motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power braking behavior for motors
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set outtake motor velocity PIDF coefficients
        outtakeMotor.setVelocityPIDFCoefficients(380, 3.37, 181, 2.914);

        // initialize AprilTag
        initAprilTag();

        // initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // Wait for the DS start button to be touched.
        telemetry.addLine("TeleOp Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //Start TeleOp gameplay loop
        while (opModeIsActive()) {

            // Transfer finite state machine with toggleable buttons and press delay
            if (gamepad1.y && e.seconds() - transTimeMarker > 0.35) {
                // stop transfer
                if (transferOn) {
                    transferOn = false;
                    intakeMotor.setPower(0);
                }
                // activate transfer
                else {
                    transferOn = true;
                    intakeMotor.setPower(0.4);
                }
                transTimeMarker = e.seconds();
            }

            // Intake finite state machine with toggleable buttons and press delay
            if (gamepad1.dpad_up && e.seconds() - intakeTimeMarker > 0.3) {
                // outtake with intake
                if (!intoutOn) {
                    intakeMotor.setPower(-0.67);
                    intoutOn = true;
                    intinOn = false;
                }
                // stop intake
                else {
                    intakeMotor.setPower(0);
                    intoutOn = false;
                }
                intakeTimeMarker = e.seconds();
            }
            else if (gamepad1.dpad_down && e.seconds() - intakeTimeMarker > 0.3) {
                // intake with intake
                if (!intinOn) {
                    intakeMotor.setPower(0.8);
                    intinOn = true;
                    intoutOn = false;
                }
                // stop intake
                else {
                    intakeMotor.setPower(0);
                    intinOn = false;
                }
                intakeTimeMarker = e.seconds();
            }
            else if (gamepad1.dpad_left) {
                intakeMotor.setPower(-0.67);
                outtakeMotorVelo = -270;
                outtakeMotor.setPower(outtakeMotorVelo);
                fixItMarker = e.seconds();
                fixItPlease = true;
            }

            // Outtake Motor finite state machine with gradual acceleration and press delay
            if (gamepad1.right_bumper && outtakeMotorVelo < 270 && e.seconds() - outtakeTimeMarker > 0.25) {
                // increase flywheel speed
                outtakeMotorVelo += 3;
                outtakeMotor.setVelocity(outtakeMotorVelo, AngleUnit.DEGREES);
                outtakeTimeMarker = e.seconds();
            }
            else if (gamepad1.left_bumper && outtakeMotorVelo > -270 && e.seconds() - outtakeTimeMarker > 0.25) {
                // decrease flywheel speed
                outtakeMotorVelo -= 3;
                outtakeMotor.setVelocity(outtakeMotorVelo, AngleUnit.DEGREES);
                outtakeTimeMarker = e.seconds();
            }
            else if (gamepad1.x) {
                // stop all systems
                outtakeMotorVelo = 0;
                transferOn = false;
                intakeMotor.setPower(0);
                outtakeMotor.setVelocity(outtakeMotorVelo, AngleUnit.DEGREES);
            }
            else if (gamepad1.a) {
                outtakeMotorVelo = -0.00315195 * Math.pow(centerToTargetVector[0], 2) + 1.66973 * centerToTargetVector[0] + 67.03349;
                outtakeMotor.setVelocity(outtakeMotorVelo, AngleUnit.DEGREES);
            }

            if (fixItPlease && e.seconds() - fixItMarker > 0.15) {
                intakeMotor.setPower(0);
                outtakeMotorVelo = 75;
                outtakeMotor.setVelocity(75);
                fixItPlease = false;
            }

            // Take controller inputs
            double y = -inputAcceleration(gamepad1.left_stick_y); // Remember, Y stick value is reversed
            double x = inputAcceleration(gamepad1.left_stick_x);
            double rx = inputAcceleration(gamepad1.right_stick_x);

            // Calculate motor powers for bot-relative drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set motor power based on above calculations
            lf.setPower(frontLeftPower);
            lr.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rr.setPower(backRightPower);

            telemetryAprilTag();

            if (centerToTargetVector[1] < 1) {
                telemetry.addLine(currentFunny + "\n");
                lockEndReady = true;
            }
            else if (lockEndReady) {
                currentFunny = ready.get((int) (Math.random() * 12));
                lockEndReady = false;
            }

            // Send flywheel motor data to telemetry
            telemetry.addLine("Applied Outtake Velo: " + outtakeMotorVelo + " deg/s");
            telemetry.addLine("Current Outtake Velo" + outtakeMotor.getVelocity()/28 + " RPM");
            telemetry.update();
        } // End of TeleOp gameplay loop

        // Stop camera
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     **/
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    /**
     * Add telemetry about AprilTag detections.
     **/
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addLine(String.format("%d AprilTags Detected", currentDetections.size()));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s\n", detection.id, detection.metadata.name));

                // telemetry to locate target behind goal AprilTags
                if (detection.id == 20 || detection.id == 24) {
                    double bearingRads = Math.toRadians(detection.ftcPose.bearing);
                    double yawRads = Math.toRadians(detection.ftcPose.yaw);

                    double centerToTargetX = detection.ftcPose.range * Math.cos(bearingRads) + targetToTagDist * Math.cos(yawRads) + camToCenterDist;
                    double centerToTargetY = detection.ftcPose.range * Math.sin(bearingRads) + targetToTagDist * Math.sin(yawRads);

                    centerToTargetVector[0] = ((int) (Math.sqrt(centerToTargetX * centerToTargetX + centerToTargetY * centerToTargetY) * 100)) / 100.0; // Center to Target Range (In)
                    centerToTargetVector[1] = ((int) (Math.toDegrees(Math.atan(centerToTargetY / centerToTargetX) * 100))) / 100.0; // Center to Target Bearing

                    telemetry.addLine(String.format("\nHeading diff from target: %.2f deg", centerToTargetVector[1]));
                    telemetry.addLine(String.format("Distance from target: %.2f in\n", centerToTargetVector[0]));
                }
                else {
                    centerToTargetVector[1] = 180;
                    centerToTargetVector[0] = 0;
                }
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)\n", detection.center.x, detection.center.y));
                centerToTargetVector[1] = 180;
                centerToTargetVector[0] = 0;
            }
        }   // end for() loop
    }


    /**
     * Input acceleration for drivetrain inputs based on parabolic curve
     */
    private double inputAcceleration(double input) {
        if (input < 0) return -0.91 * Math.pow(-input, 1.8) - 0.09;
        else if (input > 0) return 0.91 * Math.pow(input, 1.8) + 0.09;
        return 0;
    }
}
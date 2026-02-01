package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
import java.util.List;

@TeleOp
public class TeleOpDrive extends LinearOpMode   {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static ElapsedTime e = new ElapsedTime();
    final double targetToTagDist = 15; // *Perpendicular* distance between a sensed AprilTag and the target point
    final double camToCenterDist = 9.7; // *Perpendicular* distance between the camera and the robot's center of rotation

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare motors
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf"); // front right
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr"); // back right
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf"); // front left
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr"); // back left
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        CRServo transfer = hardwareMap.get(CRServo.class, "transfer");

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

        // Reverse the right side motors. This may be wrong for setup.
        // If robot moves backwards when commanded to go forwards, reverse the left side instead.
        // See the note about this earlier on this page.

        // Correct motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
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
                    transfer.setPower(0);
                    transferOn = false;
                    intakeMotor.setPower(0);
                }
                // activate transfer
                else {
                    transferOn = true;
                    transfer.setPower(0.25);
                    intakeMotor.setPower(0.75);
                }
                transTimeMarker = e.seconds();
            }

            // Intake finite state machine with toggleable buttons and press delay
            if (gamepad1.dpad_up && e.seconds() - intakeTimeMarker > 0.35) {
                // outtake with intake
                if (!intoutOn) {
                    intakeMotor.setPower(-0.8);
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
            else if (gamepad1.dpad_down && e.seconds() - intakeTimeMarker > 0.35) {
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
                transfer.setPower(0);
                outtakeMotor.setVelocity(outtakeMotorVelo, AngleUnit.DEGREES);
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

            // Send AprilTag and flywheel motor data to telemetry
            telemetryAprilTag();
            telemetry.addLine("Applied Outtake Velo: " + outtakeMotorVelo + " deg/s");
            telemetry.addLine("Current Outtake Velo" + outtakeMotor.getVelocity(AngleUnit.DEGREES) + " deg/s");
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
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));

                // telemetry to locate target behind goal AprilTags
                if (detection.id == 20 || detection.id == 24) {
                    // line between target and AprilTag + continuation of line between target and AprilTag as line between AprilTag and intersection of with triSideB
                    double triSideA = targetToTagDist + Math.abs(detection.ftcPose.x) / Math.abs(Math.cos(Math.toRadians(detection.ftcPose.yaw)));

                    // line between robot axis of rotation and camera + y distance reading + continuation of y distance reading as line between vertex of x and y distance reading and intersection with triSideA
                    double triSideB = camToCenterDist + detection.ftcPose.y + Math.abs(detection.ftcPose.x) * Math.abs(Math.tan(Math.toRadians(detection.ftcPose.yaw)));

                    // Angle between triSide A and triSideB
                    double triAngleC = Math.toRadians(90 - detection.ftcPose.yaw);

                    // Triangle Side C, in geometrical context
                    double centerToTargetRange = Math.sqrt(triSideA*triSideA + triSideB*triSideB - 2 * triSideA * triSideB * Math.cos(triAngleC));

                    // Triangle Angle A, in geometrical context
                    double centerToTargetBearing = Math.toDegrees(Math.asin(triSideA * Math.sin(triAngleC) / centerToTargetRange));

                    telemetry.addLine("Target Distance: " + centerToTargetRange + " inch");
                    telemetry.addLine("Target Heading Diff: " + centerToTargetBearing + " deg");
                }
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    /**
     * Input acceleration for drivetrain inputs based on parabolic curve
     */
    private double inputAcceleration(double input) {
        if (input < 0) return -0.91 * Math.pow(-input, 1.7) - 0.09;
        else if (input > 0) return 0.91 * Math.pow(input, 1.7) + 0.09;
        return 0;
    }
}
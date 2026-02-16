package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TeleOpDrive extends LinearOpMode   {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal; // Camera
    private static ElapsedTime e = new ElapsedTime();
    private double currentTime = 0; // Time since TeleOp start
    final double targetToTagDist = 10; // *Perpendicular* distance between AprilTag and the target point
    final double camToCenterDist = 5.25; // *Perpendicular* distance between the camera and the robot's center of rotation
    private Follower follower; // PedroPathing follower
    private static double[] centerToTargetVector = new double[2]; // 0th index stores range from robot center to target, 1st index stores bearing from robot center to target

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare motors
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        // Initialize PedroPathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
        follower.update();

        // Variables for outtake finite state machine
        double outtakeVelo = 0; // outtake motor velocity
        double outtakeMarker = 0; // time since last input

        // Variables for intake finite state machine
        double intakePower = 0; // intake motor velocity
        double intakeMarker = 0; // time since last input

        // Variables for unjamming system
        boolean unstuckOn = false; // activation state of unjamming system
        double unstuckMarker = 0; // time since last input

        // Variables for automated drive
        boolean automatedDrive = false; // activation state of automated drive
        double automationMarker = 0; // time since last input

        // Correct motor directions
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power braking behavior for motors
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set outtake motor velocity PIDF coefficients
        PIDFCoefficients coefficients = outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        coefficients.p = 67;

        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        // initialize AprilTag
        initAprilTag();

        // Wait for the driver station start button to be touched
        telemetry.addLine("TeleOp Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // start TeleOp with manual drive
        follower.startTeleopDrive();

        //Start TeleOp gameplay loop
        while (opModeIsActive()) {

            // update per-loop variables
            currentTime = e.seconds();
            follower.update();

            // manual drive input processing
            if (!automatedDrive || gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x != 0) {
                follower.setTeleOpDrive(
                        -inputAcceleration(gamepad1.left_stick_y),
                        -inputAcceleration(gamepad1.left_stick_x),
                        -inputAcceleration(gamepad1.right_stick_x) * 0.8,
                        true
                );
            }

            // Intake finite state machine
            if (gamepad1.dpad_up && currentTime - intakeMarker > 0.25) {
                intakePower = intakePower == -0.67 ? 0 : -0.67;
                intakeMotor.setPower(intakePower);

                intakeMarker = currentTime;
            } // reverse intake
            else if (gamepad1.dpad_down && currentTime - intakeMarker > 0.25) {
                intakePower = intakePower == 0.75 ? 0 : 0.75;
                intakeMotor.setPower(intakePower);

                intakeMarker = currentTime;
            } // forward intake
            else if (gamepad1.y && currentTime - intakeMarker > 0.25) {
                intakePower = intakePower == 0.5 ? 0 : 0.5;
                intakeMotor.setPower(intakePower);

                intakeMarker = currentTime;
            } // slower forward intake

            // Outtake Motor finite state machine
            if (gamepad1.right_bumper && currentTime - outtakeMarker > 0.25) {
                outtakeVelo += 10;
                outtakeMotor.setVelocity(outtakeVelo);

                outtakeMarker = currentTime;
            } // bump down outtake velocity
            else if (gamepad1.left_bumper && currentTime - outtakeMarker > 0.25) {
                outtakeVelo -= 10;
                outtakeMotor.setVelocity(outtakeVelo);

                outtakeMarker = currentTime;
            } // bump up outtake velocity
            else if (gamepad1.a && currentTime - automationMarker > 0.25) {
                if (!automatedDrive && centerToTargetVector[1] < 180) {
                    Pose currentPose = follower.getPose();
                    follower.holdPoint(new Pose(currentPose.getX(), currentPose.getPose().getY(), currentPose.getHeading() + Math.toRadians(centerToTargetVector[1])));
                    follower.resumePathFollowing();
                    follower.update();

                    outtakeVelo = 0.0121451* Math.pow(centerToTargetVector[0], 2) + 5.4271* centerToTargetVector[0] + 813;
                    outtakeMotor.setVelocity(outtakeVelo);

                    automatedDrive = true;
                }
                else {
                    follower.startTeleOpDrive();
                    automatedDrive = false;
                }

                automationMarker = currentTime;

            } // set outtake velocity based on regression, hold position and correct heading


            // Stop intake and outtake, set to manual drive
            if (gamepad1.x) {
                outtakeVelo = 0;
                outtakeMotor.setVelocity(outtakeVelo);

                intakePower = 0;
                intakeMotor.setPower(intakePower);

                follower.startTeleOpDrive();
                automatedDrive = false;
            }

            // Unjamming system
            if (gamepad1.b) {
                intakePower = -0.5;
                intakeMotor.setPower(intakePower);

                outtakeVelo = -2000;
                outtakeMotor.setVelocity(outtakeVelo);

                unstuckOn = true;
                unstuckMarker = currentTime;
            } // start unjamming system
            if (unstuckOn && currentTime - unstuckMarker > 0.2) {
                intakePower = 0;
                intakeMotor.setPower(intakePower);

                outtakeVelo = 0;
                outtakeMotor.setVelocity(outtakeVelo);

                unstuckOn = false;
            } // stop unjamming system

            telemetryAprilTag();

            // Send flywheel motor data to telemetry
            telemetry.addLine("Applied Outtake Velo: " + outtakeVelo + " tps");
            telemetry.addLine("Current Outtake Velo" + outtakeMotor.getVelocity() + "tps");
            telemetry.addLine("" + Global.kP  + "");
            telemetry.addLine("" + Global.kI  + "");
            telemetry.addLine("" + Global.kD  + "");
            telemetry.addLine("" + Global.kF  + "");
            telemetry.update();
        }
        // End of TeleOp gameplay loop

        // Stop camera
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor
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
    }

    /**
     * Add telemetry about AprilTag detections and update robot center to target vector
     **/
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addLine(String.format("%d AprilTags Detected", currentDetections.size()));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("==== (ID %d) %s\n", detection.id, detection.metadata.name));

                // telemetry to locate target behind goal AprilTags
                if (detection.id == 20 || detection.id == 24) {
                    double bearingRads = Math.toRadians(detection.ftcPose.bearing);
                    double yawRads = Math.toRadians(detection.ftcPose.yaw + (detection.id == 20 ? -8 : 8));

                    double centerToTargetX = detection.ftcPose.range * Math.cos(bearingRads) + targetToTagDist * Math.cos(yawRads) + camToCenterDist;
                    double centerToTargetY = detection.ftcPose.range * Math.sin(bearingRads) + targetToTagDist * Math.sin(yawRads);

                    centerToTargetVector[0] = ((int) (Math.sqrt(centerToTargetX * centerToTargetX + centerToTargetY * centerToTargetY) * 100)) / 100.0; // Center to Target Range (In)
                    centerToTargetVector[1] = ((int) (Math.toDegrees(Math.atan(centerToTargetY / centerToTargetX) * 100))) / 100.0; // Center to Target Bearing

                    if (centerToTargetVector[1] < -1) {
                        telemetry.addLine("Turn Right");
                    }
                    else if (centerToTargetVector[1] > 1) {
                        telemetry.addLine("Turn Left");
                    }
                    else {
                        telemetry.addLine("====| Ready! |====\n");
                    }

                    telemetry.addLine(String.format("Heading diff from target: %.2f deg", centerToTargetVector[1]));
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
        if (input < 0) return -0.9 * Math.pow(-input, 1.6) - 0.1;
        else if (input > 0) return 0.9 * Math.pow(input, 1.6) + 0.1;
        return 0;
    }
}
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
public class TeleOpDrive extends LinearOpMode {

   private AprilTagProcessor aprilTag;
   private VisionPortal visionPortal;
   private static ElapsedTime e = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf"); // front right
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr"); // back right
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf"); // front left
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr"); // back left
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        CRServo rollers = hardwareMap.get(CRServo.class, "rollers");

        double outtakeMotorPower = 0;
        double outtakeTimeMarker = 0;
        double intakeTimeMarker = 0;

        boolean intoutButtonState = false;
        boolean intinButtonState = false;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        // Correct motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

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

        while (opModeIsActive()) {

            // Rollers finite state machine
            if (gamepad1.y){
                rollers.setPower(0.5);
            }
            else if (gamepad1.a) {
                rollers.setPower(-0.5);
            }
            else {
                rollers.setPower(0);
            }



            // Intake Motor finite state machine with toggleable buttons
            if (gamepad1.dpad_up && !intoutButtonState && e.seconds() - intakeTimeMarker > 0.33) {
                intakeMotor.setPower(0.67);
                intoutButtonState = true;
                intinButtonState = false;
                intakeTimeMarker = e.seconds();
            }
            else if (gamepad1.dpad_down && !intinButtonState && e.seconds() - intakeTimeMarker > 0.33) {
                intakeMotor.setPower(-0.67);
                intinButtonState = true;
                intoutButtonState = false;
                intakeTimeMarker = e.seconds();
            }
            else if ((gamepad1.dpad_down && intinButtonState) || (gamepad1.dpad_up && intoutButtonState) && e.seconds() - intakeTimeMarker > 0.3) {
                intakeMotor.setPower(0);
                intinButtonState = false;
                intoutButtonState = false;
                intakeTimeMarker = e.seconds();
            }

            // Outtake Motor finite state machine with gradual acceleration
            outtakeMotor.setPower(outtakeMotorPower);
            if (gamepad1.left_bumper && outtakeMotorPower < 1.0 && e.seconds() - outtakeTimeMarker > 0.25) {
                outtakeMotorPower += 0.1;
                outtakeTimeMarker = e.seconds();
            }
            else if (gamepad1.right_bumper && outtakeMotorPower > -1.0 && e.seconds() - outtakeTimeMarker > 0.25) {
                outtakeMotorPower -= 0.1;
                outtakeTimeMarker = e.seconds();
            }
            else if (gamepad1.x) {
                outtakeMotorPower = 0;
            }

            // Take controller inputs
            double y = -gamepad1.left_stick_y * 0.75; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 0.75;
            double rx = gamepad1.right_stick_x * 0.75;

            // Reset Yaw
            // This button choice was made so that it is hard to hit on accident
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
                telemetry.addLine("Yaw reset");
            }

            // Take Yaw reading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute va lue) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Set motor power based on above calculations
            lf.setPower(frontLeftPower);
            lr.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rr.setPower(backRightPower);

            telemetryAprilTag();
            telemetry.addLine("Outtake Motor Power: " + outtakeMotorPower * 100 + "%");
            telemetry.update();

            // Save CPU resources; can resume streaming when needed
            /**
            if (gamepad1.dpad_down) {
                  visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                  visionPortal.resumeStreaming();
            }
             **/

            // Share the CPU
            sleep(20);
        }

        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
    **/
    private void initAprilTag() {
         // Create the AprilTag processor.
         aprilTag = new AprilTagProcessor.Builder()
                 // The following default settings are available to un-comment and edit as needed.
                 //.setDrawAxes(false)
                 //.setDrawCubeProjection(false)
                 //.setDrawTagOutline(true)
                 //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                 //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                 //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                 // == CAMERA CALIBRATION ==
                 // If you do not manually specify calibration parameters, the SDK will attempt
                 // to load a predefined calibration for your camera.
                 //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                 // ... these parameters are fx, fy, cx, cy.
                 .build();

         // Adjust Image Decimation to trade-off detection-range for detection-rate.
         // eg: Some typical detection data using a Logitech C920 WebCam
         // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
         // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
         // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
         // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
         // Note: Decimation can be changed on-the-fly to adapt during a match.
         //aprilTag.setDecimation(3);

         // Create the vision portal by using a builder.
         VisionPortal.Builder builder = new VisionPortal.Builder();

         // Set the camera
         builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

         // Choose a camera resolution. Not all cameras support all resolutions.
         //builder.setCameraResolution(new Size(640, 480));

         // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
         //builder.enableLiveView(true);

         // Set the stream format; MJPEG uses less bandwidth than default YUY2.
         //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

         // Choose whether or not LiveView stops if no processors are enabled.
         // If set "true", monitor shows solid orange screen if no processors enabled.
         // If set "false", monitor shows camera view without annotations.
         //builder.setAutoStopLiveView(false);

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
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
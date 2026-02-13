package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.15)
            .forwardZeroPowerAcceleration(-33.161346995) // done
            .lateralZeroPowerAcceleration(-68.515847) // done
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0.0003, 0.0024, 0.02)) // done
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0.005, 0.01, 0.033)) // done
            .drivePIDFCoefficients((new FilteredPIDFCoefficients(0.042,0.00015,0.0011,0.6,0.038)))
            .centripetalScaling(0.00053);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.38 , 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.4457888458)
            .yVelocity(55.06827);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("forwardEncoder")
            .strafeEncoder_HardwareMapName("strafeEncoder")
            .forwardPodY(6.125)
            .strafePodX(-0.4375)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.001999)
            .strafeTicksToInches(0.00199633)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change
                            RevHubOrientationOnRobot.UsbFacingDirection.UP // Change
                    )
            );
}

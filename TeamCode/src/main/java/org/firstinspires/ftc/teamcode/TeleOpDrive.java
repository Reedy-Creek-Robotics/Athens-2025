package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Blank Linear TeleOp", group="TeleOp")
public class TeleOpDrive extends LinearOpMode {

    private DcMotorEx coolMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        coolMotor = hardwareMap.get(DcMotorEx.class, "coolMotor");

        coolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        coolMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean bool = true;

        while (opModeIsActive()) {

            if (bool) coolMotor.setPower(1);
            else if (!bool) coolMotor.setPower(0);

            bool = false;

        }

    }

    public int givemeInt(int input) {
        return input;
    }
}
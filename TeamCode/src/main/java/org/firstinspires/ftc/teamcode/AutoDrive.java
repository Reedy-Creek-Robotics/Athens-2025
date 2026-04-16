package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="Blank Auto", group="Autonomous")
public class AutoDrive extends OpMode {
    private DcMotorEx lf, lr, rf, rr;

    private ElapsedTime t = new ElapsedTime();


    @Override
    public void init() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
    }
        // Initialize drivetrain motors

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        if (t.seconds() < 2){

            lf.setPower(0.5);
            lr.setPower(0.5);
            rf.setPower(0.5);
            rr.setPower(0.5);
        }
        else{
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
    }

    @Override
    public void stop() {}

}
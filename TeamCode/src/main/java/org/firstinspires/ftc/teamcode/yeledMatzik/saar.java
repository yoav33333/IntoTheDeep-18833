package org.firstinspires.ftc.teamcode.yeledMatzik;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class saar extends OpMode {
    DcMotorEx l;
    DcMotorEx r;

    @Override
    public void init() {
        l = hardwareMap.get(DcMotorEx.class, "l");
        r = hardwareMap.get(DcMotorEx.class, "r");
        r.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
    }
}

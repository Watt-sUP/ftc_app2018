package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Runner {
    private DcMotor leftF, leftB, rightF, rightB;

    Runner (DcMotor _m1, DcMotor _m2, DcMotor _m3, DcMotor _m4)
    {
        leftF = _m1;
        leftB = _m2;
        rightF = _m3;
        rightB = _m4;
    }
}

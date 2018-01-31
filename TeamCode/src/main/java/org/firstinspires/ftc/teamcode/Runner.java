package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Runner {
    private DcMotor leftF, leftB, rightF, rightB;

    Runner (DcMotor _leftF, DcMotor _leftB, DcMotor _rightF, DcMotor _rightB)
    {
        leftF = _leftF;
        leftB = _leftB;
        rightF = _rightF;
        rightB = _rightB;
    }
}

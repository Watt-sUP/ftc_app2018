package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Runner {
    private DcMotor leftF, leftB, rightF, rightB;
    private Telemetry.Item telemetry;
    private boolean verbose = false;
    private int error = 0;

    Runner (DcMotor _frontLeft, DcMotor _backLeft, DcMotor _frontRight, DcMotor _backRight)
    {
        leftF = _frontLeft; leftB = _backLeft; rightF = _frontRight; rightB = _backRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initialCheck();
    }

    Runner (DcMotor _frontLeft, DcMotor _backLeft, DcMotor _frontRight, DcMotor _backRight, Telemetry.Item _telemetry)
    {
        leftF = _frontLeft; leftB = _backLeft; rightF = _frontRight; rightB = _backRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry = _telemetry;
        verbose = true;
        initialCheck();
    }

    public void setMode(DcMotor.RunMode mode)
    {
        leftF.setMode(mode);
        leftB.setMode(mode);
        rightB.setMode(mode);
        rightF.setMode(mode);
    }

    public void initialCheck()
    {
        if( leftF.getController() != leftB.getController() )
        {
            telemetry.setValue("Different controller for LEFT motors!");
            error = 1;
        }

        if( rightF.getController() != rightB.getController() )
        {
            telemetry.setValue("Different controller for RIGHT motors!");
            error = 2;
        }
    }


}

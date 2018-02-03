package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CubeCollector
{
    private Servo upL, upR, downL, downR;
    private DcMotor lift;
    private Telemetry.Item telemetry;
    private boolean verbose = false;
    private int[] openP = {0, 0, 0, 0};
    private int[] closeP = {0, 0, 0, 0};
    private int[] liftP = {0, (int)(1e6)};
    private int whichServos = 0;

    /// TODO: get values for openP, closeP and liftP

    CubeCollector(Servo _upL, Servo _upR, Servo _downL, Servo _downR, DcMotor _lift)
    {
        upL = _upL; upR = _upR; downL = _downL; downR = _downR; lift = _lift;
        verbose = false;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    CubeCollector(Servo _upL, Servo _upR, Servo _downL, Servo _downR, DcMotor _lift, Telemetry.Item _telemetry)
    {
        upL = _upL; upR = _upR; downL = _downL; downR = _downR; lift = _lift;
        telemetry = _telemetry;
        verbose = true;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.setValue("OK!");
    }

    public void moveLift(double power)
    {
        if(lift.getCurrentPosition() < liftP[0])   { lift.setPower(0); return; }
        if(lift.getCurrentPosition() > liftP[1])   { lift.setPower(0); return; }

        lift.setPower(power);
    }

    public void setLiftMode(DcMotor.RunMode rm) { lift.setMode(rm); }

    public void setControllingServo(int x)
    {
        whichServos = x;
    }

    public void closeServo()
    {
        if( (whichServos & 1) > 0 )
        {
            upL.setPosition(closeP[0]);
            upR.setPosition(closeP[1]);
        }
        if( (whichServos & 2) > 0 )
        {
            downL.setPosition(closeP[2]);
            downR.setPosition(closeP[3]);
        }
    }

    public void openServo()
    {
        if( (whichServos & 1) > 0 )
        {
            upL.setPosition(openP[0]);
            upR.setPosition(openP[1]);
        }
        if( (whichServos & 2) > 0 )
        {
            downL.setPosition(openP[2]);
            downR.setPosition(openP[3]);
        }
    }


}

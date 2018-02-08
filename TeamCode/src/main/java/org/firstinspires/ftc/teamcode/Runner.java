package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Runner {
    private DcMotor leftF, leftB, rightF, rightB;
    private Telemetry.Item telemetry;
    private boolean verbose = false;    /// True if telemetry is initialized
    private int error = 0;  /// Error code.

    Runner (DcMotor _frontLeft, DcMotor _backLeft, DcMotor _frontRight, DcMotor _backRight)
    {
        leftF = _frontLeft; leftB = _backLeft; rightF = _frontRight; rightB = _backRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verbose = false;
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

    public void setMode(DcMotor.RunMode mode)   /// Sets mode for all motors
    {
        leftF.setMode(mode);
        leftB.setMode(mode);
        rightB.setMode(mode);
        rightF.setMode(mode);
    }

    public void initialCheck()  /// Checks for errors before init
    {
        if( leftF.getController() != leftB.getController() )
        {
            if(verbose) telemetry.setValue("Different controller for LEFT motors!");
            error = 1;
        }

        if( rightF.getController() != rightB.getController() )
        {
            if(verbose) telemetry.setValue("Different controller for RIGHT motors!");
            error = 2;
        }

        if(error == 0 && verbose)  telemetry.setValue("OK!");
    }

    public void setPower(double left, double right, double ratio)   /// Sets power to motors
    {
        left *= ratio; right *= ratio;
        leftF.setPower(left); leftB.setPower(left);
        rightF.setPower(right); rightB.setPower(right);
    }

    public void setPower(double left, double right)
    {
        setPower(left, right, 1.0);
    }

    public void move(double y, double x, double r)  /// Moves robot considering gamepad axis Y(forward-backward), X(rotation), R(power ratio)
    {
        if(x == 0)
            setPower(-y, y, r);
        else if(y == 0)
            setPower(-x, -x, r);
        else if(x < 0)
            setPower(-y - y * x, y, r);
        else if(x > 0)
            setPower(-y, y - y * x, r);
    }

    public void move(double y, double x)
    {
        move(y, x, 1.0);
    }

    public void logInformation(String info) /// Logs information to telemetry
    {
        if(!verbose)    return;
        if(info == "Power") telemetry.setValue( leftF.getPower() + " " + leftB.getPower() + " " + rightF.getPower() + " " + rightB.getPower() );
        if(info == "Power2") telemetry.setValue( leftF.getPower() + " " + rightF.getPower() );
        if(info == "Position") telemetry.setValue( leftF.getCurrentPosition() + " " + leftB.getCurrentPosition() + " " + rightF.getCurrentPosition() + " " + rightB.getCurrentPosition() );
    }
}

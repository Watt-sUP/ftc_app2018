package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Runner class is used for controlling the moving system of the robot #Mugurel
 */
public class Runner {
    /**
     * leftF = front left motor
     * leftB = back left motor
     * rightF = front right motor
     * rightB = back right motor
     */
    private DcMotor leftF, leftB, rightF, rightB;

    /**
     * telemetry = telemetry item assigned to the moving system
     * verbose = true if any telemetry item is assigned to this class
     */
    private Telemetry.Item telemetry;
    private boolean verbose = false;

    /**
     * Error code. More than 0 if an error is detected in the initial check.
     */
    private int error = 0;

    /**
     * Gamepad values
     */
    private double rnrX, rnrY, rnrR;

    /**
     *
     */
    private double ticksPerRevolution = 1220;
    private double wheelDiameter = 4.0 * 2.54;
    private double wheelLength = wheelDiameter * Math.PI;

    /**
     * Constructor
     * @param _frontLeft front left motor
     * @param _backLeft back left motor
     * @param _frontRight front right motor
     * @param _backRight back right motor
     * @param _telemetry OPTIONAL -> telemetry item
     */
    Runner (DcMotor _frontLeft, DcMotor _backLeft, DcMotor _frontRight, DcMotor _backRight, Object... _telemetry)
    {
        leftF = _frontLeft; leftB = _backLeft; rightF = _frontRight; rightB = _backRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(_telemetry.length > 0 && (_telemetry[0] instanceof Telemetry.Item))
        {
            verbose = true;
            telemetry = (Telemetry.Item) _telemetry[0];
        }
        else
            verbose = false;

        initialCheck();

        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);
        rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightB.setDirection(DcMotorSimple.Direction.REVERSE);

        ticksPerRevolution = leftF.getMotorType().getTicksPerRev();
    }

    /**
     * Checks for errors before initialization
     */
    private void initialCheck()
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

    /**
     * Set the run mode for all motors
     * @param mode the new RunMode
     */
    public void setMode(DcMotor.RunMode mode)
    {
        leftF.setMode(mode);
        leftB.setMode(mode);
        rightB.setMode(mode);
        rightF.setMode(mode);
    }


    /**
     * Set power for all motors, for left and right sides
     * @param left left side power
     * @param right right side power
     * @param ratio scale ratio
     */
    public void setPower(double left, double right, double ratio)
    {
        left *= ratio; right *= ratio;
        leftF.setPower(left); leftB.setPower(left);
        rightF.setPower(right); rightB.setPower(right);
    }

    /**
     * Set power for all motors, for left and right sides, without any scale ratio
     * @param left left side power
     * @param right right side power
     */
    public void setPower(double left, double right)
    {
        setPower(left, right, 1.0);
    }

    /**
     * Moves robot according to gamepad axis Y(forward-backward), X(rotation) and scale ratio R(power ratio)
     * @param y forward-backward axis
     * @param x rotation axis
     * @param r scale ratio
     */
    public void move(double y, double x, double r)
    {
        rnrX = x; rnrY = y; rnrR = r;

        if(x == 0)
            setPower(-y, y, r);
        else if(y == 0)
            setPower(-x, -x, r);
        else if(x < 0)
            setPower(-y - y * x, y, r);
        else if(x > 0)
            setPower(-y, y - y * x, r);
    }

    /**
     * Moves robot according to gamepad axis Y(forward-backward), X(rotation) and no scale ratio
     * @param y forward-backward axis
     * @param x rotation axis
     */
    public void move(double y, double x)
    {
        move(y, x, 1.0);
    }

    /**
     * Moves robot cm centimeters forward (> 0) or backward (< 0)
     * @param cm number of centimeters
     */
    public void distanceMove(double cm, double power)
    {
        double addTicks = cm / wheelLength * ticksPerRevolution;
        int add = (int)Math.round(addTicks);

        DcMotor.RunMode oldMode = leftF.getMode();

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftF.setTargetPosition( leftF.getCurrentPosition() - add );
        leftB.setTargetPosition( leftB.getCurrentPosition() - add );
        rightF.setTargetPosition( rightF.getCurrentPosition() + add );
        rightB.setTargetPosition( rightB.getCurrentPosition() + add );

        power = Math.abs(power);
        setPower(power, power);

        while( leftF.isBusy()||rightF.isBusy()||leftB.isBusy()||rightB.isBusy());

        setPower(0.0, 0.0);

        setMode(oldMode);
    }

    /**
     * Logs information to telemetry if verbose is true
     * @param info information type
     */
    public void logInformation(String info)
    {
        if(!verbose)    return;
        if(info.equals("Power")) telemetry.setValue( leftF.getPower() + " " + leftB.getPower() + " " + rightF.getPower() + " " + rightB.getPower() );
        if(info.equals("Power2")) telemetry.setValue( leftF.getPower() + " " + rightF.getPower() );
        if(info.equals("Position")) telemetry.setValue( leftF.getCurrentPosition() + " " + leftB.getCurrentPosition() + " " + rightF.getCurrentPosition() + " " + rightB.getCurrentPosition() );
        if(info.equals("Gamepad"))   telemetry.setValue( String.format("%.3f", rnrX) + " " + String.format("%.3f", rnrY) + " " + String.format("%.3f", rnrR) );
    }
}

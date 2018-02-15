package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * CubeCollector class is used for controlling the cube collecting system of the robot #Mugurel
 */
public class CubeCollector
{
    /**
     * 0. upL = up left servo
     * 1. upR = up right servo
     * 2. downL = down left servo
     * 3. downR = down right servo
     */
    private Servo upL, upR, downL, downR;

    /**
     * lift = lift motor
     */
    private DcMotor lift;

    /**
     * telemetry = telemetry item assigned to the moving system
     * verbose = true if any telemetry item is assigned to this class
     */
    private Telemetry.Item telemetry;
    private boolean verbose = false;

    /**
     * openP = constant values of the open positions of servos
     * closeP = constant values of the closed positions of servos
     * midP = constant values of the middle positions of servos
     */
    private double[] openP = {0.55, 0.35, 0.2, 0.75};
    private double[] closeP = {0.2, 0.6, 0.55, 0.45};
    private double[] midP = {0.5, 0.5, 0.5, 0.5};

    /**
     * liftP = lower and upper positions for the lift motor
     * (for Duta not breaking the ropes)
     */
    private int[] liftP = {0, (int)(1e6)};

    /**
     * stateUp = actual state of the upper servos
     * stateDown = actual state of the lower servos
     * 0 = open, 1 = closed
     */
    private int stateUp = 0, stateDown = 0;

    /// TODO: get values for openP, closeP and liftP

    /**
     * Constructor
     * @param _upL up left servo
     * @param _upR up right servo
     * @param _downL down left servo
     * @param _downR down right servo
     * @param _lift lift motor
     * @param _telemetry OPTIONAL -> telemetry item
     */
    CubeCollector(Servo _upL, Servo _upR, Servo _downL, Servo _downR, DcMotor _lift, Object... _telemetry)
    {
        upL = _upL; upR = _upR; downL = _downL; downR = _downR; lift = _lift;

        if(_telemetry.length > 0 && (_telemetry[0] instanceof Telemetry.Item))
        {
            telemetry = (Telemetry.Item) _telemetry[0];
            verbose = true;
            telemetry.setValue("OK!");
        }
        else
            verbose = false;

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upL.setPosition(openP[0]);
        upR.setPosition(openP[1]);
        downL.setPosition(openP[2]);
        downR.setPosition(openP[3]);
    }

    /**
     * Set power to lift motor
     * Optional: motor can't go upper/lower than some limits
     * @param power new power of the lift motor
     */
    public void moveLift(double power)
    {
        /// TODO: Get values and set limits
        //if(lift.getCurrentPosition() < liftP[0])   { lift.setPower(0); return; }
        //if(lift.getCurrentPosition() > liftP[1])   { lift.setPower(0); return; }

        lift.setPower(power);
    }

    /**
     * Set run mode for lift motor
     * @param rm the new RunMode
     */
    public void setLiftMode(DcMotor.RunMode rm) { lift.setMode(rm); }

    /**
     * Change state of the servos (open - closed)
     * @param ids ID-s of the servos
     *            1 = upper servos
     *            2 = lower servos
     *            3 = upper + lower servos
     */
    public void changeState(int ids)
    {
        if( (ids & 1) > 0 )
        {
            if(stateUp == 1)
            {
                upL.setPosition(openP[0]);
                upR.setPosition(openP[1]);
            }
            else if(stateUp == 0)
            {
                upL.setPosition(closeP[0]);
                upR.setPosition(closeP[1]);
            }
            stateUp ^= 1;
        }
        if( (ids & 2) > 0 )
        {
            if(stateDown == 1)
            {
                downL.setPosition(openP[2]);
                downR.setPosition(openP[3]);
            }
            else if(stateDown == 0)
            {
                downL.setPosition(closeP[2]);
                downR.setPosition(closeP[3]);
            }
            stateDown ^= 1;
        }
    }

    /**
     * Set servos to middle position
     * @param ids ID-s of the servos
     *            1 = upper servos
     *            2 = lower servos
     *            3 = upper + lower servos
     */
    public void goMiddle(int ids)
    {
        if( (ids & 1) > 0 )
        {
            upL.setPosition(midP[0]);
            upR.setPosition(midP[1]);
        }
        if( (ids & 2) > 0 )
        {
            downL.setPosition(midP[2]);
            downR.setPosition(midP[3]);
        }
    }

    /**
     * PRIVATE
     * Changes position of a servo adding val to the current position
     * @param servo the changing servo
     * @param val added value
     */
    private void addValue(Servo servo, double val)
    {
        double p = servo.getPosition();
        p += val;
        p = Math.max(0.0, p);
        p = Math.min(1.0, p);
        servo.setPosition(p);
    }

    /**
     * Changes position of a servo adding val to the current position
     * @param servo id of the changing servo
     * @param val added value
     */
    public void addValue(int servo, double val)
    {
        if(servo == 0)  addValue(upL, val);
        if(servo == 1)  addValue(upR, val);
        if(servo == 2)  addValue(downL, val);
        if(servo == 3)  addValue(downR, val);
    }

    /**
     * Enables/Disables power in all servos in this servo controller.
     * @param val 0 = disabled, 1 = enabled
     */
    public void setPower(int val)
    {
        if(val == 0)    upL.getController().pwmDisable();
        if(val == 1)    upL.getController().pwmEnable();
    }

    /**
     * Logs information to telemetry if verbose is true
     * @param info information type
     */
    public void logInformation(String info)
    {
        if(info == "Positions")     telemetry.setValue( String.format("%.3f", upL.getPosition()) + " " + String.format("%.3f", upR.getPosition()) + " " +
                                                        String.format("%.3f",downL.getPosition()) + " " + String.format("%.3f", downR.getPosition()) + " " +
                                                        lift.getCurrentPosition() );
    }
}

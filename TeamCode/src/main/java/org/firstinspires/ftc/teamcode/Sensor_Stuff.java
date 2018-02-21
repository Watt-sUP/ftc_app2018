package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Tavi on 2/21/2018.
 */

public class Sensor_Stuff {

    Runner rnr;

    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor u_s;
    ColorSensor cs;


    Sensor_Stuff  (ModernRoboticsI2cRangeSensor Ultrasonic_Distance, ModernRoboticsI2cGyro Gyro, ColorSensor cs, Runner x ){
        gyro = Gyro;
        u_s = Ultrasonic_Distance;
        cs = cs;
        rnr = x;
    }
    private void  Keep_Orientation  (int Optimal_pos ){
        while( gyro.getHeading() != Optimal_pos )
            //ToDo: test if  rotation constant is big enough for rotation to be made when Optimal_pos-gyro.getHeading is small, try a bigger constant
         if( Optimal_pos-gyro.getHeading() > 0 )
            rnr.setPower(( Optimal_pos - gyro.getHeading() ) * 0.72 ,-( Optimal_pos - gyro.getHeading() ) * 0.72);//proportional rotation
            else if( Optimal_pos-gyro.getHeading() < 0 )
             rnr.setPower(( Optimal_pos - gyro.getHeading() ) * 0.72 ,( Optimal_pos - gyro.getHeading() ) * 0.72);//proportional rotation
        }

}

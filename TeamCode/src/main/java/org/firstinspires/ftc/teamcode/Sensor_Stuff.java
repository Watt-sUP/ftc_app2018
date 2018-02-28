package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Tavi on 2/21/2018.
 */

public class Sensor_Stuff {

    private Runner rnr;
    private boolean dist_s_Target=false, dist_offset=false ;
    private ModernRoboticsI2cGyro gyro;
    private ModernRoboticsI2cRangeSensor dist_s;
    private ColorSensor c_s;


    Sensor_Stuff  (ModernRoboticsI2cRangeSensor Ultrasonic_Distance, ModernRoboticsI2cGyro Gyro, ColorSensor cs, Runner x ){
        gyro = Gyro;
        dist_s= Ultrasonic_Distance;
        c_s = cs;
        rnr = x;

    }
    private void  Keep_Orientation  (int Optimal_pos ){
        while( gyro.getHeading() != Optimal_pos )
            // test to rotate to the smaller angle
                //ratio
            //ToDo: test if  rotation constant is big enough for rotation to be made when Optimal_pos-gyro.getHeading is small, try a bigger/smaller constant
         if( Optimal_pos-gyro.getHeading() > 0 )
            rnr.setPower(-( Optimal_pos - gyro.getHeading() ) * 0.72 ,-( Optimal_pos - gyro.getHeading() ) * 0.72);//proportional rotation
            else if( Optimal_pos-gyro.getHeading() < 0 )
             rnr.setPower(-( Optimal_pos - gyro.getHeading() ) * 0.72 ,( Optimal_pos - gyro.getHeading() ) * 0.72);//proportional rotation
        }

    /**
     // place cube where you need to
     * @param drawer_target_pos specifies where the cube will be placed 1-right, 2-centre, 3-right
     * ToDo: test to set drawer_target_pos type to vumark types instead of int, so the function to be called straight with the vumark output
     *ToDo: set distance to look for drawer
     */
    private void Place_Cube  ( int drawer_target_pos) {
        int nr = 0;
        while ( true ) {
            if ( dist_s.getDistance( DistanceUnit.CM ) < 10 && !dist_s_Target )
            {
                dist_offset=false;
                dist_s_Target = true;
                nr++;
                rnr.setPower(-0.6,0.6);
                if (nr == drawer_target_pos){
                    rnr.distanceMove(10,0.4);
                    break;
                }

            
            if ( dist_s.getDistance(DistanceUnit.CM) > 10 && !dist_offset ) dist_offset = true;
            if ( dist_offset ) dist_s_Target = false;
            }

        }

    }
}

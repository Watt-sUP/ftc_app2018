package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Blue Side", group="Linear Opmode")
//@Disabled
public class AutonomousBlueSide extends AutonomousFunctions {

    static
    {
        color = 1;
        forward = -1;
    }

    @Override
    public void runOpMode() {
        // DECLARE THE EXTENDER SERVO ,CORESPONDING TO JEWEL DROPPING ARM SO THAT WE COULD SET THE INIT POSITION TO A STRAIGHT ONE (1.0)
        Servo extender;
        extender=hardwareMap.get(Servo.class,"extender");
        extender.setPosition(1.0);
        initialization ();

        waitForStart();

        runtime.reset();

        /// Grab cube
        state.setValue("grab cube");
        telemetry.update();
        grab_cube();
        if (!opModeIsActive()) return;

        /// Score jewels
        state.setValue("score jewels");
        telemetry.update();
        scoreJewels(extender);
        if(!opModeIsActive())   return;

        /// Gets key drawer
        state.setValue("get key drawer");
        int drawer = getKeyDrawer();
        if(forward == 1)    drawer = 4 - drawer;
        telemetry.update();

        /// Get down from platform
        state.setValue("get down from platform");
        telemetry.update();
        getDown();
        if(!opModeIsActive())   return;

        /// Rotate 90 degrees
        state.setValue("rotate 1st time");
        telemetry.update();
        Keep_Orientation(270);
        if(!opModeIsActive())   return;

        /// Get back some cm to be sure that we not miss the drawer
        state.setValue("get back");
        telemetry.update();
        rnr.distanceMove(forward * -10, 0.4, this);
        if(!opModeIsActive())   return;

        /// Go in front of first drawer
        state.setValue("go to drawer");
        telemetry.update();
        go_to_drawer();
        if (!opModeIsActive()) return;

        /// Go to needed drawer
        state.setValue("pick drawer");
        telemetry.update();
        pick_drawer(2);

        /// Rotate 90 degrees
        state.setValue("rotate");
        telemetry.update();
        Keep_Orientation(180);
        if( !opModeIsActive() ) return;

        /// Place cube
        state.setValue("place cube");
        telemetry.update();
        place_cube_encoders(false);
        if( !opModeIsActive() ) return;
    }
}
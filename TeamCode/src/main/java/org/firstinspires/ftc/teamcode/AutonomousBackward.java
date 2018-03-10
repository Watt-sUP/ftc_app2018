package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Backward", group="Linear Opmode")
//@Disabled

public class AutonomousBackward extends AutonomousMiddle
{
    static
    {
        forward = -1;
        color = 0;
    }
}

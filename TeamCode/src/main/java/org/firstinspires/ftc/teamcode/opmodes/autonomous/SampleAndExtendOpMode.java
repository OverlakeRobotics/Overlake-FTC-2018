package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Sample and Extend", group = "Competition")
public class SampleAndExtendOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        initSystems();
        waitForStart();
        delatch();
        sample();
        driveSystem.turn(90, 1);
        runSlideOut();
    }
}

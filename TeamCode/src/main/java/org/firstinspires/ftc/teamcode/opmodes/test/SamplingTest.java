package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomousOpMode;

@Autonomous(name = "Sample Testing", group = "Test")
public class SamplingTest extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        initSystems();
        waitForStart();
        tensorFlow.activate();
    }
}

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "The Null Special", group = "Null")
public class TheNullSpecialOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        initSystems();
        waitForStart();
        delatch();
        sample();
        driveToDepot();
        retrace();
    }

    public void driveToDepot() {
        if (getCubePos() != 0)
        {
            driveSystem.turn(45, 1);
        }
        driveSystem.driveToPositionInches(10, 0.7);
        markerSystem.place();
        sleep(500);
        markerSystem.reset();
    }

    public void retrace() {
        switch (getCubePos()) {
            case 1:
                driveSystem.turn(-45, 1);
                driveSystem.driveToPositionInches(10, -1);
                driveSystem.turn(33, 1);
                break;
            case 2:
                driveSystem.turn(-45, 1);
                driveSystem.driveToPositionInches(10, -1);
                driveSystem.turn(53, 1);
                break;
        }
        driveSystem.driveToPositionInches(32, -1);
        driveSystem.turnAbsolute(0, 1);
    }
}

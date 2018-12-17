package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Crater Start ~!NO DE-LATCH!~", group = "Competition")
public class CraterStartNoDelatchOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        this.initSystems();
        telem("init Finished", 0.1);
        waitForStart();
        sample();
        driveSystem.driveToPositionInches(backCubeIn, -1);
        distanceSystem.strafeTowardWallPolar(inFromWall, cratApproachDeg1, cratToWallHeading, autonoPower);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        driveSystem.driveToPositionInches(40, -1);
        driveSystem.driveToPositionInches(15, -0.5);
        markerSystem.place();
        sleep(500);
        markerSystem.reset();
        driveSystem.driveToPositionInches(70, autonoPower, false);
        driveSystem.turnAbsolute(42, 1);
        runSlideOut();
    }
}

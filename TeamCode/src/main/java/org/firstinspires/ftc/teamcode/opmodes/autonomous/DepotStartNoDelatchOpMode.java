package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Depot Start ~!NO DE-LATCH!~", group = "Competition")
public class DepotStartNoDelatchOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

        ////
        waitForStart();
        ////
        // tensor flow

        //driveSystem.mecanumDriveXY(35.1563665,-90.0525999);

        sample();
        int cubePos = getCubePos();
        if (cubePos == 0) {
            driveSystem.turnAbsolute((-90 +  depApproachDeg0), autonoPower);
        } else if (cubePos == 2) {
            driveSystem.turnAbsolute((-90 - depApproachDeg0), autonoPower);
        }
        driveSystem.driveToPositionInches(25, 1);
        driveSystem.driveToPositionInches(10, 0.5);
        telem("passed ParkInDepot", 0.01);
        distanceSystem.strafeTowardWallPolar(inFromWall, depApproachDeg1, depToWallHeading, autonoPower);
        markerSystem.place();
        sleep(500);
        driveSystem.driveToPositionInches(70, autonoPower, false);
        driveSystem.turnAbsolute(87, 1);
        markerSystem.reset();
        runSlideOut();
        stop();
    }
}

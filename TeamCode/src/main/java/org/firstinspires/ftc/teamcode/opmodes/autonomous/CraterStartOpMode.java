package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Crater Start", group = "Competition")
public class CraterStartOpMode extends BaseAutonomousOpMode {
    private final String TAG = "CraterStartOpMode";

    @Override
    public void run() {
        this.initSystems();
        telem("init Finished", 0.1);
        waitForStart();
        delatch();
        sample();
        driveSystem.driveToPositionInches(backCubeIn, -1);
        distanceSystem.strafeTowardWallPolar(inFromWall, cratApproachDeg1, cratToWallHeading, autonoPower);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        parkInDepot(-autonoPower, colorSystem);
        markerSystem.place();
        sleep(300);
        markerSystem.reset();
        driveSystem.driveToPositionInches(67, autonoPower, false);
        driveSystem.turnAbsolute(42, 1);
        runSlideOut();
    }

    // cube in middle: knocks right ball
    // cube on right or left: thinks cube is center
    // cube on right: knocks off left ball while strafing
    // random (mainly cube right?): runs into wall when aligning
}

// backCubeIn, cratTargDist3, inFromWall, toWallPow
// cratApproachDeg0, cratApproachDeg1, cratToWallHeading
// autonoPower, toWallPow
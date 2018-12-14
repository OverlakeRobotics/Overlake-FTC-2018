package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CraterStartOpMode", group = "Bot")
public class CraterStartOpMode extends BaseAutonomousOpMode {
    private final String TAG = "CraterStartOpMode";

    public CraterStartOpMode () {
        super("CraterStartOpMode");
    }

    @Override
    public void runOpMode() {
        this.initSystems();
        waitForStart();
        delatch();
        sample();
        distanceSystem.strafeTowardWallPolar(inFromWall, cratApproachDeg1, cratToWallHeading, autonoPower);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        parkInDepot(-autonoPower, colorSystem);
        markerSystem.place();
        sleep(700);
        markerSystem.reset();
        driveSystem.driveToPositionInches(75, autonoPower, false);
        driveSystem.turnAbsolute(42, 1);
        runSlideOut();
    }
}

// backCubeIn, cratTargDist3, inFromWall, toWallPow
// cratApproachDeg0, cratApproachDeg1, cratToWallHeading
// autonoPower, toWallPow
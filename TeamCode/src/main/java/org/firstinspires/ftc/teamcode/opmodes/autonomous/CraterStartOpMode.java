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

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

        ////
        waitForStart();
        ////

        // tensor flow
        delatch();
        sample();
        driveSystem.driveToPositionInches(backCubeIn, -autonoPower, false);

        // either this code
        //driveSystem.turnAbsolute(cratApproachDeg0, autonoPower);
        //driveSystem.driveToPositionInches(cratTargDist3, -autonoPower, false);
        //distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);

        // or this code
        distanceSystem.strafeTowardWallPolar(inFromWall, cratApproachDeg1, cratToWallHeading, autonoPower);


        parkInDepot(-autonoPower, colorSystem);
        markerSystem.place();
        sleep(700);
        markerSystem.reset();
        driveSystem.driveToPositionInches(50, autonoPower, false);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        parkOnCrator(toWallPow, initPitch, initRoll);
        markerSystem.reset();


        stop();
    }
}

// backCubeIn, cratTargDist3, inFromWall, toWallPow
// cratApproachDeg0, cratApproachDeg1, cratToWallHeading
// autonoPower, toWallPow
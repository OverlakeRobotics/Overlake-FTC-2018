package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

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
        driveSystem.driveToPositionInches(backCubeIn, -1, false);
        driveSystem.turnAbsolute(approachDeg0, 1);
        distanceSystem.getCloseToWall(targDist1, 1);
        //driveSystem.turnAbsolute(approachDeg1, 1);
        //distanceSystem.getCloseToWall(7, 0.8);
        driveSystem.turnAbsolute(approachDeg2, 1);
        parkInDepot(1, colorSystem);
        markerSystem.place();
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.BACKWARD);
        distanceSystem.driveAlongWallInches(40, 3, 6, 1, true);
        parkOnCrator(1, initPitch, initRoll);

        stop();
    }
}

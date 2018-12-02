package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
public class EducayshunOpMode extends BaseAutonomousOpMode {
    private final String TAG = "EducationOpMode";
    private double initPitch;
    private double initRoll;

    public int zone;
    private int backCubeIn;
    private int approachDeg0;
    private int approachDeg1;
    private int approachDeg2;
    private double targDist1;

    public EducayshunOpMode() {
        super("EducayshunOpMode");

         zone = config.getInt("zone");
         backCubeIn = config.getInt("backCubeIn");//10
         approachDeg0 = config.getInt("approachDeg0");//-90
        approachDeg1 = config.getInt("approachDeg1");//-120
        config.getInt("approachDeg2");//-135
        config.getDouble("targDist1");



        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();
    }

    @Override
    public void runOpMode()
    {

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

        ////
        waitForStart();
        ////

        // 0 is red crator

        if (zone == 0) {
            // tensor flow
            driveSystem.driveToPositionInches(backCubeIn, -1, false);
            driveSystem.turnAbsolute(approachDeg0, 1);
            distanceSystem.getCloseToWall(targDist1, 1);
            //driveSystem.turnAbsolute(approachDeg1, 1);
            //distanceSystem.getCloseToWall(7, 0.8);
            driveSystem.turnAbsolute(approachDeg2, 1);
            driveSystem.parkInDepot(1, colorSystem);
            markerSystem.place();
            driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.BACKWARD);
            distanceSystem.driveAlongWallInches(40, 3, 6, 1, true);
            driveSystem.parkOnCrator(1);
        } else if(zone == 1) {

        } else if (zone == 2) {

        } else if (zone == 3) {

        }

        distanceSystem.getCloseToWall(7, -0.7);

        sleep(8000);
        telemetry.addLine("reached wall");
        telemetry.update();

        distanceSystem.driveAlongWallToCrater(3, 6, 1);

        sleep(3000);

        distanceSystem.driveAlongWallToDepot(3, 6, -1);


        //driveSystem.driveToPositionInches(30, 1);

        /*int ticksLeft = 3000;
        ExponentialRamp ramp = new ExponentialRamp(new Point(0, 0.1), new Point(ticksLeft, 1));

        int counts = 6;
        for (int i = 0; i < counts; i++) {
            telemetry.addLine("ticksLeft: " + ticksLeft);
            telemetry.addLine("rampX: " + ramp.scaleX(ticksLeft));
            telemetry.update();
            ticksLeft -= (ticksLeft / 6);
            sleep(1000);
        }*/


        //driveSystem.driveToPositionInches(15, 1);

        /*for (int i = 0; i < 50000; i++) {
            //distanceSystem.telemetry();
            colorSystem.telemetry();
            sleep(10);
        }*/

        // FR BL BR FL

        stop();
    }

    private void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}
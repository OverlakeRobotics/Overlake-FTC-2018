package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
    public class EducayshunOpMode extends BaseAutonomousOpMode {
        private final String TAG = "EducationOpMode";

        public EducayshunOpMode() {
            super("EducayshunOpMode");
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
                parkInDepot(1, colorSystem);
                markerSystem.place();
                driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.BACKWARD);
                distanceSystem.driveAlongWallInches(40, 3, 6, 1, true);
                parkOnCrator(1, initPitch, initRoll);
            } else if(zone == 1) {

            } else if (zone == 2) {

            } else if (zone == 3) {

            }

            /*distanceSystem.getCloseToWall(7, -0.7);

            sleep(8000);
            telemetry.addLine("reached wall");
            telemetry.update();

            distanceSystem.driveAlongWallToCrater(3, 6, 1);

            sleep(3000);

            distanceSystem.driveAlongWallToDepot(3, 6, -1);


            //driveSystem.driveToPositionInches(30, 1);*/

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
}
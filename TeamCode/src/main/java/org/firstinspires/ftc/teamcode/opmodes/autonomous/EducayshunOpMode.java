package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.base.DriveSystem4Wheel;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
public class EducayshunOpMode extends BaseAutonomousOpMode {

    public EducayshunOpMode() {
        super("EducayshunOpMode");
    }

    @Override
    public void runOpMode()
    {

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + disSys), 0.25);

        ////
        waitForStart();
        ////

        disSys.driveAlongWallInches(60, 3, 6, 1);

        //driveSystem.turn(90, 1);

        /*driveSystem.motorFrontLeft.setPower(1);
        sleep(1000);
        driveSystem.motorFrontLeft.setPower(0);
        sleep(1000);
        driveSystem.motorFrontRight.setPower(1);
        sleep(1000);
        driveSystem.motorFrontRight.setPower(0);
        sleep(1000);
        driveSystem.motorBackRight.setPower(1);
        sleep(1000);
        driveSystem.motorBackRight.setPower(0);
        sleep(1000);
        driveSystem.motorBackLeft.setPower(1);
        sleep(1000);
        driveSystem.motorBackLeft.setPower(0);
        sleep(1000);*/

        driveSystem.setDirection(MecanumDriveSystem.MecanumDriveDirection.STRAFE_LEFT);
        driveSystem.setPower(0.8);
        sleep(5000);



        //driveSystem.driveToPositionInches(15, 1);

        /*for (int i = 0; i < 5000; i++) {
            disSys.telemetry();
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
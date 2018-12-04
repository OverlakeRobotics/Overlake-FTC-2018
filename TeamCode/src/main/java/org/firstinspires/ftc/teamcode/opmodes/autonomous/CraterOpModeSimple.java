package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.marker.Marker;
import org.firstinspires.ftc.teamcode.systems.tensorflow.TensorFlow;

@Autonomous(name = "Crater OpMode")
public class CraterOpModeSimple extends LinearOpMode
{
    private static final int DISTANCE_TO_DEPOT = 36;

    private static final int SCREEN_WIDTH = 1280;
    private static final int SCREEN_CENTER = 1280 / 2;
    private static final int OFFSET = 50;

    private TensorFlow tensorFlow;
    private MecanumDriveSystem driveSystem;
    private Marker markerServo;

    @Override
    public void runOpMode() {
        initializeOpMode();
        waitForStart();
        driveSystem.driveToPositionInches(50, 0.6);

    }
    private void initializeOpMode() {
        driveSystem = new MecanumDriveSystem(this);
    }
}
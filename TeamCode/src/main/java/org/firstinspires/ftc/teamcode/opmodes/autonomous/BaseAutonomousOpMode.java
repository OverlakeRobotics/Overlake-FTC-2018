package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.configs.ConfigParser;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.lidar.LidarNavigationSystem;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;
import org.firstinspires.ftc.teamcode.systems.marker.Marker;

/**
 * Created by EvanCoulson on 10/11/17.
 */

public abstract class BaseAutonomousOpMode extends LinearOpMode
{
    ConfigParser config;
    public MecanumDriveSystem driveSystem;
    public IMUSystem imuSystem;
    public ColorSystem colorSystem;
    public LidarNavigationSystem distanceSystem;
    public Marker markerSystem;

    public double initPitch;
    public double initRoll;

    public int backCubeIn; // the inches to back up after knocking the cube
    public int cratApproachDeg0; //the angle at which the robot approaches the wall after tensor flow
    public int cratApproachDeg1; // reletive to starting position
    public int approachDeg2;
    public double cratTargDist1;
    public int cratTargDist2;
    public int cratTargDist3;
    public int cratTargDist4;

    public int depDepappraochIn; // depot(OpMode) depot approach inches
    public double depWallHeading;
    public int depToCratIn;

    public double toWallPow;
    public double autonoPower;
    public int inFromWall;
    public int zone;

    public double CRITICAL_ANGLE = 1.5;
    int RED_TRGGER_VALUE = 12;
    int BLUE_TRIGGER_VALUE = 8;

    public BaseAutonomousOpMode(String opModeName)
    {
        //config = new ConfigParser(opModeName + ".omc");
        config = new ConfigParser("Autonomous.omc");
        telemetry.setMsTransmissionInterval(200);
    }

    protected void initSystems()
    {
        this.driveSystem = new MecanumDriveSystem(this);
        this.imuSystem = new IMUSystem(this);
        colorSystem = new ColorSystem(this);
        distanceSystem = new LidarNavigationSystem(this, driveSystem, colorSystem);
        markerSystem = new Marker(this);

        backCubeIn = config.getInt("backCubeIn");//10

        cratApproachDeg0 = config.getInt("cratApproachDeg0");//-90
        cratApproachDeg1 = config.getInt("cratApproachDeg1");//-120
        cratTargDist1 = config.getDouble("cratTargDist1");
        cratTargDist2 = config.getInt("cratTargDist2");
        cratTargDist3 = config.getInt("cratTargDist3");
        cratTargDist4 = config.getInt("cratTargDist4");

        depDepappraochIn = config.getInt("depDepApproachIn");
        depWallHeading = config.getDouble("depWallHeading");
        depToCratIn = config.getInt("depToCratIn");

        toWallPow = config.getDouble("ToWallPow");
        autonoPower = config.getDouble("autonopower");
        inFromWall = config.getInt("inFromWall");
        zone = config.getInt("zone");

        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();
    }

    public void parkInDepot(double maxPower, ColorSystem colorSystem) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while ((colorSystem.getRed() < RED_TRGGER_VALUE) &&
                (colorSystem.getBlue() < BLUE_TRIGGER_VALUE)) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void parkOnCrator(double maxPower, double initPitch, double initRoll) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while (((Math.abs(imuSystem.getPitch() - initPitch) < CRITICAL_ANGLE) ||
                (Math.abs(imuSystem.getRoll() - initRoll) < CRITICAL_ANGLE))) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }

    public int determineBlockPos() {
        int pos = 0;
        if ((imuSystem.getHeading() - driveSystem.initialHeading) >= 10) {
            pos = 0;
        } else if ((imuSystem.getHeading() - driveSystem.initialHeading) <= 10) {
            pos = 2;
        } else {
            pos = 1;
        }
        telemetry.addLine("block pos: " + pos);
        telemetry.update();
        return  pos;
    }
}

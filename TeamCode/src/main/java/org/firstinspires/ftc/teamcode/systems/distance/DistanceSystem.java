package org.firstinspires.ftc.teamcode.systems.distance;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.base.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.base.System;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;

public class DistanceSystem extends System {
    DistanceSensor lidar;
    DistanceSensor lidar2;
    Rev2mDistanceSensor sensorTimeOfFlight;
    HardwareMap hwmap;
    MecanumDriveSystem driveSystem;
    IMUSystem imu;
    ColorSystem colorSystem;

    double initPitch;
    double initRoll;

    public DistanceSystem(OpMode opMode, MecanumDriveSystem ds, IMUSystem iMu, ColorSystem cs) {
        super(opMode, "DriveSystem4Wheel");
        this.hwmap = opMode.hardwareMap;
        driveSystem = ds;
        imu = iMu;
        colorSystem= cs;
        initSystem();

        initPitch = imu.getpitch();
        initRoll = imu.getRoll();
    }

    public void initSystem() {
        lidar = hwmap.get(DistanceSensor.class, "sensor_range");
        lidar2 = hwmap.get(DistanceSensor.class, "sensor_range2");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)lidar;
    }


    public double getFrontRightDistance() {
        double d1 = getDistance1();
        double d2 = getDistance2();
        double dp = 9; //distance between
        //return d2 * Math.cos((1/Math.tan(((d2 - d1) * dp))));
        return d1;
    }

    public double getDistance1() {
        //telemetry.log("lidar1", ("" + lidar));
        //telemetry.log("range", String.format("%.01f in", lidar.getDistance(DistanceUnit.INCH)));
        telemetry.write();
        return lidar.getDistance(DistanceUnit.INCH);
    }

    public double getDistance2() {
        //telemetry.log("lidar2", ("" + lidar2));
        //telemetry.log("range", String.format("%.01f in", lidar2.getDistance(DistanceUnit.INCH)));
        telemetry.write();
        return lidar2.getDistance(DistanceUnit.INCH);
    }

    public void telemetry() {
        // generic DistanceSensor methods.
        telemetry.log("lidar1", ("" + lidar));
        telemetry.log("lidar2", ("" + lidar2));
        telemetry.log("range", String.format("%.01f in", lidar.getDistance(DistanceUnit.INCH)));
        telemetry.log("shaerposte: ", "" + getFrontRightDistance());


        telemetry.log("MecanumDriveSystem","disp 12, power 1.0, PowerX: ");

        telemetry.write();
    }

    public double getDistanceTopRight() {
        double d1 = getDistance1();
        double d2 = getDistance2();
        double df = d1 - d2;
        double ds = 10.0; // inches
        double d = (d1 * (Math.sin(Math.atan((ds/df)))));
        return d;
    }

    public void driveAlongWallInches(int inches, double closeBuffer, double farBuffer, double power) {
        int ticks = driveSystem.inchesToTicks(inches);
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
        driveSystem.motorFrontRight.setPower(0);

        driveSystem.motorFrontRight.setTargetPosition(driveSystem.motorFrontRight.getCurrentPosition() + ticks);
        driveSystem.motorFrontLeft.setTargetPosition(driveSystem.motorFrontLeft.getCurrentPosition() + ticks);
        driveSystem.motorBackRight.setTargetPosition(driveSystem.motorBackRight.getCurrentPosition() + ticks);
        driveSystem.motorBackLeft.setTargetPosition(driveSystem.motorBackLeft.getCurrentPosition() + ticks);

        driveSystem.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveSystem.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveSystem.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveSystem.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp ramp = new ExponentialRamp(new Point(0, driveSystem.RAMP_POWER_CUTOFF),
                new Point((ticks / 4), power));

        double adjustedPower = Range.clip(power, -1.0, 1.0);

        driveSystem.motorBackRight.setPower(adjustedPower);
        driveSystem.motorBackLeft.setPower(adjustedPower);
        driveSystem.motorFrontLeft.setPower(adjustedPower);
        driveSystem.motorFrontRight.setPower(adjustedPower);

        while (driveSystem.motorFrontLeft.isBusy() ||
                driveSystem.motorFrontRight.isBusy() ||
                driveSystem.motorBackRight.isBusy() ||
                driveSystem.motorBackLeft.isBusy()) {

            if ((getDistance1() <= closeBuffer) ||
                    (getDistance2() <= closeBuffer) ||
                    (getDistance1() >= farBuffer) ||
                    (getDistance2() >= farBuffer)) {
                telemetry.log("driveTest", "distance buffer triggered");

                driveSystem.setPower(0);
                
                while ((getDistance1() >= farBuffer) ||
                        (getDistance2() >= farBuffer) ||
                        (getDistance1() <= closeBuffer) ||
                        (getDistance2() <= closeBuffer)) {

                    double turnPower = (power / 2);
                    double rightPower = turnPower;
                    double leftPower = turnPower;
                    if ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        } else {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        }
                    } else if ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        } else {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        }
                    } else if ((getDistance1() >= farBuffer) || getDistance2() <= closeBuffer) {
                        telemetry.log("driveTest", "turning RIGHT");
                        rightPower = 0;
                        leftPower = turnPower;
                    } else if ((getDistance2() >= farBuffer) || (getDistance1() <= closeBuffer)) {
                        telemetry.log("driveTest", "turning LEFT");
                        rightPower = turnPower;
                        leftPower = 0;
                    }

                    telemetry.log("driveTest", "looping correction R-power: " + rightPower);
                    telemetry.log("driveTest", "looping correction R-power: " + leftPower);

                    telemetry.log("MecanumDriveSystem","power motorFL: " + driveSystem.motorFrontLeft.getPower());
                    telemetry.log("MecanumDriveSystem","power motorFR: " + driveSystem.motorFrontRight.getPower());
                    telemetry.log("MecanumDriveSystem","power motorBL: " + driveSystem.motorBackLeft.getPower());
                    telemetry.log("MecanumDriveSystem","power motorBR: " + driveSystem.motorBackRight.getPower());
                    telemetry.write();

                    driveSystem.tankDrive(leftPower, rightPower);

                }
            }
            int distance = driveSystem.getMinDistanceFromTarget();

            if (distance < 50) {
                break;
            }

            telemetry.log("MecanumDriveSystem","targetPos motorFL: " + driveSystem.motorFrontLeft.getTargetPosition());
            telemetry.log("MecanumDriveSystem","targetPos motorFR: " + driveSystem.motorFrontRight.getTargetPosition());
            telemetry.log("MecanumDriveSystem","targetPos motorBL: " + driveSystem.motorBackLeft.getTargetPosition());
            telemetry.log("MecanumDriveSystem","targetPos motorBR: " + driveSystem.motorBackRight.getTargetPosition());

            telemetry.log("MecanumDriveSystem","currentPos motorFL: " + driveSystem.motorFrontLeft.getCurrentPosition());
            telemetry.log("MecanumDriveSystem","currentPos motorFR: " + driveSystem.motorFrontRight.getCurrentPosition());
            telemetry.log("MecanumDriveSystem","currentPos motorBL: " + driveSystem.motorBackLeft.getCurrentPosition());
            telemetry.log("MecanumDriveSystem","currentPos motorBR: " + driveSystem.motorBackRight.getCurrentPosition());

            double direction = 1.0;
            if (distance < 0) {
                distance = -distance;
                direction = -1.0;
            }

            double scaledPower = ramp.scaleX(distance);
            telemetry.log("MecanumDriveSystem","power: " + scaledPower);
            driveSystem.setPower(direction * scaledPower);
            telemetry.log("MecanumDriveSystem","power motorFL: " + driveSystem.motorFrontLeft.getPower());
            telemetry.log("MecanumDriveSystem","power motorFR: " + driveSystem.motorFrontRight.getPower());
            telemetry.log("MecanumDriveSystem","power motorBL: " + driveSystem.motorBackLeft.getPower());
            telemetry.log("MecanumDriveSystem","power motorBR: " + driveSystem.motorBackRight.getPower());
            telemetry.write();
        }
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorFrontRight.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
    }

    public void driveAlongWallToCrater(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
        driveSystem.motorFrontRight.setPower(0);

        driveSystem.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);

        driveSystem.motorBackRight.setPower(adjustedPower);
        driveSystem.motorBackLeft.setPower(adjustedPower);
        driveSystem.motorFrontLeft.setPower(adjustedPower);
        driveSystem.motorFrontRight.setPower(adjustedPower);

        while (driveSystem.motorFrontLeft.isBusy() ||
                driveSystem.motorFrontRight.isBusy() ||
                driveSystem.motorBackRight.isBusy() ||
                driveSystem.motorBackLeft.isBusy()) {

            if ((getDistance1() <= closeBuffer) ||
                    (getDistance2() <= closeBuffer) ||
                    (getDistance1() >= farBuffer) ||
                    (getDistance2() >= farBuffer)) {
                telemetry.log("driveTest", "distance buffer triggered");

                driveSystem.setPower(0);

                while ((getDistance1() >= farBuffer) ||
                        (getDistance2() >= farBuffer) ||
                        (getDistance1() <= closeBuffer) ||
                        (getDistance2() <= closeBuffer)) {

                    double turnPower = (power / 2);
                    double rightPower = turnPower;
                    double leftPower = turnPower;
                    if ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        } else {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        }
                    } else if ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        } else {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        }
                    } else if ((getDistance1() >= farBuffer) || getDistance2() <= closeBuffer) {
                        telemetry.log("driveTest", "turning RIGHT");
                        rightPower = 0;
                        leftPower = turnPower;
                    } else if ((getDistance2() >= farBuffer) || (getDistance1() <= closeBuffer)) {
                        telemetry.log("driveTest", "turning LEFT");
                        rightPower = turnPower;
                        leftPower = 0;
                    }
                    telemetry.write();
                    driveSystem.tankDrive(leftPower, rightPower);

                }
            }

            if (isOnCrater()) {
                break;
            }

            driveSystem.setPower(adjustedPower);
            telemetry.write();
        }
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorFrontRight.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
    }

    public void driveAlongWallToDepot(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
        driveSystem.motorFrontRight.setPower(0);

        driveSystem.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);

        driveSystem.motorBackRight.setPower(adjustedPower);
        driveSystem.motorBackLeft.setPower(adjustedPower);
        driveSystem.motorFrontLeft.setPower(adjustedPower);
        driveSystem.motorFrontRight.setPower(adjustedPower);

        while (driveSystem.motorFrontLeft.isBusy() ||
                driveSystem.motorFrontRight.isBusy() ||
                driveSystem.motorBackRight.isBusy() ||
                driveSystem.motorBackLeft.isBusy()) {

            if ((getDistance1() <= closeBuffer) ||
                    (getDistance2() <= closeBuffer) ||
                    (getDistance1() >= farBuffer) ||
                    (getDistance2() >= farBuffer)) {
                telemetry.log("driveTest", "distance buffer triggered");

                driveSystem.setPower(0);

                while ((getDistance1() >= farBuffer) ||
                        (getDistance2() >= farBuffer) ||
                        (getDistance1() <= closeBuffer) ||
                        (getDistance2() <= closeBuffer)) {

                    double turnPower = (power / 2);
                    double rightPower = turnPower;
                    double leftPower = turnPower;
                    /*if ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        } else {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        }
                    } else if ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer)) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        } else {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        }
                    }*/
                    if (((getDistance1() >= farBuffer) || getDistance2() <= closeBuffer) ||
                            ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer))) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        } else {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        }
                    } else if (((getDistance2() >= farBuffer) || (getDistance1() <= closeBuffer)) ||
                            ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer))) {
                        if (power > 0) {
                            telemetry.log("driveTest", "turning LEFT");
                            rightPower = turnPower;
                            leftPower = 0;
                        } else {
                            telemetry.log("driveTest", "turning RIGHT");
                            rightPower = 0;
                            leftPower = turnPower;
                        }
                    }

                    telemetry.write();
                    driveSystem.tankDrive(leftPower, rightPower);

                }
            }

            if (isInDepot()) {
                break;
            }

            driveSystem.setPower(adjustedPower);
            telemetry.write();
        }
        driveSystem.motorBackLeft.setPower(0);
        driveSystem.motorBackRight.setPower(0);
        driveSystem.motorFrontRight.setPower(0);
        driveSystem.motorFrontLeft.setPower(0);
    }

    public boolean isOnCrater() {
        double criticalAngle = 1.5;
        return ((Math.abs(imu.getpitch() - initPitch) < criticalAngle) ||
                (Math.abs(imu.getRoll() - initRoll) < criticalAngle));
    }

    public boolean isInDepot() {
        int redTriggerValue = 12;
        int blueTriggerValue = 8;
        return ((colorSystem.getRed() < redTriggerValue) ||
                (colorSystem.getBlue() < blueTriggerValue));
    }

    /*public void parkInDepot(double maxPower, ColorSystem colorSystem) {
        int redTriggerValue = 12;
        int blueTriggerValue = 8;
        setDirection(DriveDirection.FORWARD);
        setPower(maxPower);

        while ((colorSystem.getRed() < redTriggerValue) &&
                (colorSystem.getBlue() < blueTriggerValue)) {
            setPower(maxPower);
        }
        setPower(0);
    }*/

    /*public void parkOnCrater(double maxPower) {
        double initPitch = imuSystem.getpitch();
        double initRoll = imuSystem.getRoll();
        double criticalAngle = 1.5;

        setDirection(DriveDirection.FORWARD);
        setPower(maxPower);

        while ((Math.abs(imuSystem.getpitch() - initPitch) < criticalAngle) &&
                (Math.abs(imuSystem.getRoll() - initRoll) < criticalAngle)) {
            setPower(maxPower);
        }
        setPower(0);
    }*/
}

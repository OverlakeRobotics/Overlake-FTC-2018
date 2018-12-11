package org.firstinspires.ftc.teamcode.systems.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;

public class AutonomousArm extends ArmSystem
{
    private final double DelatchVoltage = 1.8;
    private final double ReleaseVoltage = 0.3;
    private final double MaxGroundVoltage = 1.5;
    private final double MinGroundVoltage = 1.7;

    /**
     * Builds a new Arm System for the given opmode
     *
     * @param opMode the opmode that the arm system is currently running on
     */
    public AutonomousArm(OpMode opMode)
    {
        super(opMode);
    }

    public void delatch(MecanumDriveSystem driveSystem)
    {
        releaseArmPin();
        while (!inCollapsePosition())
        {
            runMotors(ArmDirection.DOWN, 1);
        }
        while (!hasDelatched())
        {
            if (shouldRunWheels()) {
                driveSystem.mecanumDriveXY(0.3, 0);
            }
            runMotors(ArmDirection.DOWN, 0.2);
        }
        stop();
    }

    private boolean inCollapsePosition()
    {
        return potentiometer.getVoltage() <= ReleaseVoltage;
    }

    private boolean hasDelatched()
    {
        return potentiometer.getVoltage() >= DelatchVoltage;
    }

    public void latch(MecanumDriveSystem driveSystem)
    {
        while (!inCollapsePosition())
        {
            if (shouldRunWheels()) {
                driveSystem.mecanumDriveXY(-0.3, 0);
            }
            runMotors(ArmDirection.DOWN, 0.6);
        }
        setArmPin();
        stop();
    }

    public void collapse() {
        while (!inCollapsePosition()) {
            runMotors(ArmDirection.DOWN, 0.5);
        }
    }

    public boolean shouldRunWheels() {
        return potentiometer.getVoltage() <= MaxGroundVoltage &&
                potentiometer.getVoltage() >= MinGroundVoltage;
    }
}

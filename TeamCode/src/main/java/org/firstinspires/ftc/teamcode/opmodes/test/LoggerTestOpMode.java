package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.debuggers.LinearOpModeDebugger;
import org.firstinspires.ftc.teamcode.systems.logging.ILogger;
import org.firstinspires.ftc.teamcode.systems.logging.PhoneLogger;
import org.firstinspires.ftc.teamcode.systems.logging.RobotHubLogger;

import java.util.Scanner;

@Autonomous(name = "LoggingTestOpMode")
public class LoggerTestOpMode extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHubLogger logger = new RobotHubLogger();
        waitForStart();
        logger.info("Arm System", "Hello world!");
        logger.error("Arm System", "Total failure");
        logger.stop();
    }
}

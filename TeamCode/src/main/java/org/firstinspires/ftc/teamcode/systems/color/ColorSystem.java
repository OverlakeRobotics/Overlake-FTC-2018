package org.firstinspires.ftc.teamcode.systems.color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.systems.base.System;



    public class ColorSystem extends System {

        private ColorSensor colorSensor;

        public ColorSystem(OpMode opMode) {
            super(opMode, "ColorSystem");

            colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
            colorSensor.enableLed(true);
        }

        public void telemetry() {
            log.info("Alpha", colorSensor.alpha());
            log.info("Red  ", colorSensor.red());
            log.info("Green", colorSensor.green());
            log.info("Blue ", colorSensor.blue());
        }

        public int getRed() {
            return colorSensor.red();
        }

        public int getBlue() {
            return colorSensor.blue();
        }

        public int getGreen() {
            return colorSensor.green();
        }

        public int getAlpha() {
            return colorSensor.alpha();
        }
    }

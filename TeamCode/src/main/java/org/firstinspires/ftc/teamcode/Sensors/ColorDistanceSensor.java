package org.firstinspires.ftc.teamcode.Sensors;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotModule.java
 *
 * note: the color sensor is not accurate and needs calibrate, it is current measuring distance using the amount of red or blue light that the sleeves reflects
 * the class that contains some basic functions for a robot module
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.9
 * @Version v0.0.3
 * */
public class ColorDistanceSensor {
    /** whether the sensor is looking for a blue or red sleeve, 1 for red and 0 for blue */
    private static short redOrBlueSleeves;
    /** the strength of the current environment light */
    private static final int environmentLuminosity = 30;
    /** the minimum amount of extra light(in comparison to the environment light) for the sensor to think that it detected an object */
    private static final int minActivateLuminosity = 10;
    /** the  amount of extra light received(in comparison to the environment light) for the sensor to think that the object is close enough for capturing */
    private static final int startCaptureLuminosity = 400;

    /** the color sensor located in the front of the robot's claw */
    private static ColorSensor colorSensor;

    /** whether the target is in visible, indicated using the result from the sensor */
    private boolean targetInRange;

    /** the distance to the targeted, from the best distance to grab the sleeve to the current distance between the robot and the sleeve, positive direction is to the front, 0 is the best position to grab */
    private double distanceToTarget;

    /**
     * create a color distance sensor reader, given a color sensor instance
     * @param colorSensor the specified color sensor instance
     * @param redOrBlueSleeves whether the sensor is looking for a blue or red sleeve, 1 for red and 0 for blue
     * */
    public ColorDistanceSensor(ColorSensor colorSensor, short redOrBlueSleeves) {
        this.colorSensor = colorSensor;
        redOrBlueSleeves = redOrBlueSleeves;
        targetInRange = false;
        distanceToTarget = 0;
    }
    /** automatically select color sensor */
    public ColorDistanceSensor(HardwareMap hardwareMap, short redOrBlueSleeves) {
        ColorSensor defaultColorSensor = hardwareMap.get(ColorSensor.class, "color");
        new ColorDistanceSensor(defaultColorSensor, redOrBlueSleeves);
    }

    /** get whether the target is in visible, indicated using the result from the sensor */
    public boolean isTargetInRange() {
        updateSensorReading();
        return targetInRange;
    }

    /**
     * get the distance to the sleeve in front
     * @return the distance between the robot and the sleeve, positive direction is to the front, 0 is the best position to grab, and 1 is the maximum distance */
    public double getDistanceToTarget() {
        updateSensorReading();
        return distanceToTarget;
    }

    private void updateSensorReading() {
        int sensorReading;
        if (redOrBlueSleeves == 1) sensorReading = colorSensor.red();
        else sensorReading = colorSensor.blue();

        targetInRange = sensorReading > minActivateLuminosity;
        // TODO process the distance to target using a linear mapping function
    }
}

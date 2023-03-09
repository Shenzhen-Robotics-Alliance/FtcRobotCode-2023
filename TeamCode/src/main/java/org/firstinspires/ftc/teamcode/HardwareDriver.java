/*
 * Copyright (c) 2017 FIRST. All rights reserved.
 * FileName: HardWareDriver.java
 *
 * the program that connects to the hardware of the robot
 *
 * @Author First Inspires
 * @Date 2017.Unknown
 * @Version Unknown
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareDriver
{
    /* Public OpMode members. */
    //motor
    public DcMotorEx leftFront  = null;
    public DcMotorEx leftRear   = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear  = null;

    public DcMotorEx lift_left = null;
    public DcMotorEx lift_right = null;

    //Servo
    public Servo claw = null;


    //CRServo

    /* HardwareMap. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareDriver(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define Motors
        leftFront = hwMap.get(DcMotorEx.class, "leftfront");
        leftRear = hwMap.get(DcMotorEx.class, "leftrear");
        rightFront = hwMap.get(DcMotorEx.class, "rightfront");
        rightRear = hwMap.get(DcMotorEx.class, "rightrear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift_left = hwMap.get(DcMotorEx.class, "lifter");
        lift_right = hwMap.get(DcMotorEx.class, "lifter_right");

        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);

        //lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront = hwMap.get(DcMotorEx.class,"leftFront");
        leftRear = hwMap.get(DcMotorEx.class,"leftRear");
        rightFront = hwMap.get(DcMotorEx.class,"rightFront");
        rightRear = hwMap.get(DcMotorEx.class,"rightRear");

        // Define Servos
        claw = hwMap.get(Servo.class, "tipperhopper");

        // Define CRServos

        // Init Motor Direction
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Init CRServo Direction

    }
}

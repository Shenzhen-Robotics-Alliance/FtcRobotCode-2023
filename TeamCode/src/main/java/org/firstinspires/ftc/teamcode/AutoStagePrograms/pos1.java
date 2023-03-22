/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: pos1.java
 *
 * auto stage program
 * the robot moves to position 1 by the end
 * the pilot selects the position manually to temporarily replace signal sleeves
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */
package org.firstinspires.ftc.teamcode.AutoStagePrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;

import java.util.HashMap;

@Autonomous(name = "pos1")
public class pos1 extends Roboseed_AutoStage{
    /** go to the selected sector */
    @Override
    private void proceedGoToSector() {

    }
}
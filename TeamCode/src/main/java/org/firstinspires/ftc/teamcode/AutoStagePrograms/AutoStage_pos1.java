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

/**
 * go to sector1 by the end of auto-stage program
 * using the temporary auto stage program for now
 * */
@Autonomous(name = "1_blue_right")
public class AutoStage_pos1 extends Roboseed_AutoStage_tmp{
    /** set the parking sector to be the selected */
    @Override
    short determineParkingSector() {
        return 1;
    }
}
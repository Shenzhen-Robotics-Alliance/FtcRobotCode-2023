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

@Autonomous(name = "pos1")
public class AutoStage_pos1 extends Roboseed_AutoStage{
    /** set the parking sector to be the selected */
    @Override
    short determineParkingSector() {
        return 1;
    }
}
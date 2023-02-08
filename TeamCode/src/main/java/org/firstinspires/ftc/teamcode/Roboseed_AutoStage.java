package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation;

@TeleOp(name = "AutoStateProgram_v1.0")
public class Roboseed_AutoStage extends LinearOpMode {
    ElapsedTime elaspsedTime = new ElapsedTime();

    ComputerVisionFieldNavigation fieldNavigation;

    @Override
    public void runOpMode() throws InterruptedException {
        fieldNavigation = new ComputerVisionFieldNavigation(hardwareMap);
        waitForStart();
        elaspsedTime.reset();
        if (opModeIsActive()) {
            // find the top stick and place the goal
            while(true) {
                // TODO test and debug field navigation system
                System.out.print(fieldNavigation.getRobotPosition()[0]);
                System.out.print(" ");
                System.out.print(fieldNavigation.getRobotPosition()[1]);
                System.out.print(" ");
                System.out.println(fieldNavigation.getRobotPosition()[2]);
            }
        }

    }
}

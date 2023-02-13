package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;

/*
* the robot starts in the corner of the field.
* first, the robot moves out of the parking spot and rotates 90 degree to face the navigation marks,
* the robot moves to position(according to camera) -1022, -782
*
* */

@TeleOp(name = "AutoStateProgram_v1.0")
public class Roboseed_AutoStage extends LinearOpMode {
    ElapsedTime elapsedTime = new ElapsedTime();

    ComputerVisionFieldNavigation_v2 fieldNavigation;

    @Override
    public void runOpMode() throws InterruptedException {
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        Thread fieldNavigationThread = new Thread(fieldNavigation);

        waitForStart();

        fieldNavigationThread.start();
        elapsedTime.reset();
        while(opModeIsActive()) {
            System.out.print(fieldNavigation.getRobotPosition()[0]);
            System.out.print(" ");
            System.out.print(fieldNavigation.getRobotPosition()[1]);
            System.out.print(" ");
            System.out.println(fieldNavigation.getRobotPosition()[2]);
        } fieldNavigation.terminate();
    }
}

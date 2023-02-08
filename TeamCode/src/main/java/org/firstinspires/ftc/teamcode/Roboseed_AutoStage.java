package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation;
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;

@TeleOp(name = "AutoStateProgram_v1.0")
public class Roboseed_AutoStage extends LinearOpMode {
    ElapsedTime elaspsedTime = new ElapsedTime();

    ComputerVisionFieldNavigation_v2 fieldNavigation;

    @Override
    public void runOpMode() throws InterruptedException {
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        Thread fieldNavigationThread = new Thread(fieldNavigation);

        waitForStart();

        fieldNavigationThread.start();
        elaspsedTime.reset();
        while(opModeIsActive()) {
            // TODO test and debug field navigation system, make the navigation system return whether it have visual of any marks, and complete the task using these informations
            // I guess it's about the problem of thread priority
            System.out.print(fieldNavigation.getRobotPosition()[0]);
            System.out.print(" ");
            System.out.print(fieldNavigation.getRobotPosition()[1]);
            System.out.print(" ");
            System.out.println(fieldNavigation.getRobotPosition()[2]);
        } fieldNavigation.terminate();
    }
}

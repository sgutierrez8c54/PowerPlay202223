/*
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectoryTest extends LinearOpMode {
    @Autonomous
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(RobotHardware);

        waitForStart();


        Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0));
                drive.forward(100)
                .build();
        drivetrain.followTrajectory(goForward);

        Trajectory lineToPosition = drivetrain.trajectoryBuilder(new Pose2d(10, 0, 0));
                .splineTo(new Vector2d(0,0))
                .build();
        drivetrain.followTrajectory(lineToPosition);




    }
}

*/
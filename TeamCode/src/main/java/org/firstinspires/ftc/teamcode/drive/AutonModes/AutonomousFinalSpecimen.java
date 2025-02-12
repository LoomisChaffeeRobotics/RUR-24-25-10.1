package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
@Config

public class AutonomousFinalSpecimen extends LinearOpMode {
    IMU imu;
    ElapsedTime timer;
    ElapsedTime timer2;
    SampleMecanumDrive drive;
    servoWork servos;
    boolean startedDriving = false;
    int parkSide = -1;
    TrajectorySequence specimenSide;
    TrajectorySequence specimenSide2; //post localization

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(24, -64.5, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .turn(-PI/2)
                .forward(3)
                .splineToConstantHeading(new Vector2d(5, -36), Math.toRadians(0))
//                .turn(PI/2)
//                .forward(3)
//                .addDisplacementMarker(() -> {
//                    servos.linearUpSubmersible();
//                    servos.clawOpen();
//                })
//                .back(3)
//                .addDisplacementMarker(() -> {
//                    servos.linearDownFullFromSubmersible();
//                })
//
//
//                //We might make another trajectory right here in order to re-adjust the pose of the robot
//                .splineToConstantHeading(new Vector2d(24,-50),Math.toRadians(90))
//                .turn(-PI/2)
//                .strafeRight(5)
//                .forward(5)
//                .addDisplacementMarker(() -> {
//                    servos.armDown();
//                    servos.clawClosed();
//                    servos.armUp();
//                })
//                .splineToConstantHeading(new Vector2d(-5,-38), Math.toRadians(0))
//                .turn(PI/2)
//                .forward(5)
//                .addDisplacementMarker(() -> {
//                    servos.linearUpSubmersible();
//                    servos.clawOpen();
//                    servos.linearDownFullFromSubmersible();
//                })
                    .build();
            drive.followTrajectorySequence(trajSeq);
            requestOpModeStop();
        }
    }
}


package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
@Config

public class AutonomousFinalSample extends LinearOpMode {
    SampleMecanumDrive drive;
    servoWork servos;
    Pose2d startPose = new Pose2d(-24,-60,Math.toRadians(90));
    boolean startedDriving = false;
    TrajectorySequence specimenSide;
    TrajectorySequence specimenSide2; //post localization
    TrajectorySequence sampleSide;
    public static double distance2Spec = 10d; //can be changed, as I predict this will be the variable most needing change

    @Override
    public void runOpMode() throws InterruptedException {
        if (opModeInInit()) {
            drive = new SampleMecanumDrive(hardwareMap);
            servos = new servoWork();
            servos.init(hardwareMap);
        }

        drive.setPoseEstimate(startPose);

        sampleSide = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-49,-47), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //servoWork.armDown();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                })
                .splineTo(new Vector2d(-53,-53), 5*PI/4)
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    //servoWork.arm45();
                    //servoWork.clawOpen();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
                .splineTo(new Vector2d(-55,-47), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //servoWork.armDown();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                })
                .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    //servoWork.arm45();
                    //servoWork.clawOpen();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
                .splineTo(new Vector2d(-57,-47), Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    //servoWork.armDown();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                })
                .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    //servoWork.arm45();
                    //servoWork.clawOpen();
                    //servoWork.clawClosed();
                    //servoWork.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
//
                .build();


        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(sampleSide);
        }
    }
}


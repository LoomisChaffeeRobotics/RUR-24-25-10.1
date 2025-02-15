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

public class AutonomousFinalSample extends LinearOpMode {
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
        servos = new servoWork();
        servos.init(hardwareMap);
        Pose2d startPose = new Pose2d(-24, -64.5, Math.toRadians(90));
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-24,-64.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-48,-48), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    servos.armDown();
                    servos.clawToggle();
                    servos.armUp();
                })
                .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                .turn(Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    servos.arm45();
                    servos.clawToggle();
                    servos.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
                .splineToConstantHeading(new Vector2d(-56,-47), Math.toRadians(90))
                .turn(Math.toRadians(-135))
                .addDisplacementMarker(() -> {
                    servos.armDown();
                    servos.clawToggle();
                    servos.armUp();
                })
                .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                .turn(Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    servos.arm45();
                    servos.clawToggle();
                    servos.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
                .splineToConstantHeading(new Vector2d(-56,-42), Math.toRadians(90))
                .turn(Math.toRadians(-95))
                .addDisplacementMarker(() -> {
                    servos.armDown();
                    servos.clawToggle();
                    servos.armUp();
                })
                .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                .turn(Math.toRadians(95))
                .addDisplacementMarker(() -> {
                    //servoWork.linearUpHigh();
                    servos.arm45();
                    servos.clawToggle();
                    servos.armUp();
                    //servoWork.linearDownFullFromHigh();
                })
//
                .build();
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested()) {

            drive.followTrajectorySequenceAsync(trajSeq);
            servos.armUpdate();


        }

    }
}


package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(5, -33), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //servoWork.doStuff;
                })
                .forward(3)
                .addDisplacementMarker(() -> {
                    //servoWork.domorestuff
                })
                .splineTo(new Vector2d(24,-61), Math.toRadians(180))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //servoWork.pickupthing
                })
                .splineTo(new Vector2d(-5,-33), Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
    }
}


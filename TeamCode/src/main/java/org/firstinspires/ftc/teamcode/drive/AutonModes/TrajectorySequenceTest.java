package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoWork servos = new servoWork();
        Pose2d startPose = new Pose2d(24, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(3)
                .turn(PI/2)
                .forward(3)
                .splineTo(new Vector2d(5, -33), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    servos.linearUpSubmersible();
                })
                .forward(3)
                .addDisplacementMarker(() -> {
                    servos.linearDownALittleBit();
                    servos.clawOpen();
                })
                .splineTo(new Vector2d(24,-61), Math.toRadians(180))
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


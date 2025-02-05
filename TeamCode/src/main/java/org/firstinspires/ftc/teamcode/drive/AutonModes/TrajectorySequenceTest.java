package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
public class TrajectorySequenceTest extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoWork servos = new servoWork();
        Pose2d startPose = new Pose2d(24, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(3)
                .turn(-PI / 2)
                .forward(3)
                .splineToConstantHeading(new Vector2d(5, -36), Math.toRadians(90))
                .turn(PI / 2)
                .forward(3)
                .addDisplacementMarker(() -> {
//                            servoWork.linearUpSubmersible();
//                            servoWork.clawOpen();
//                            servoWork.linearDownFullFromSubmersible();
                })
                .back(3)
                //We might make another trajectory right here in order to re-adjust the pose of the robot
                .splineToConstantHeading(new Vector2d(24, -50), Math.toRadians(90))
                .turn(-PI / 2)
                .strafeRight(5)
                .forward(5)
                .addDisplacementMarker(() -> {
//                            servoWork.armDown();
//                            servoWork.clawClosed();
//                            servoWork.armUp();
                })
                .splineToConstantHeading(new Vector2d(-5, -38), Math.toRadians(0))
                .turn(PI / 2)
                .forward(5)
                .addDisplacementMarker(() -> {
//                            servoWork.linearUpSubmersible();
//                            servoWork.clawOpen();
//                            servoWork.linearDownFullFromSubmersible();
                })
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
    }
}


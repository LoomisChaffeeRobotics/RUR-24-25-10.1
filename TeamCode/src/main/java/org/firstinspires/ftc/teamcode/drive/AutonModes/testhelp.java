package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

@Config
@Autonomous
public class testhelp extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoWork servos;
        servos = new servoWork();
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
//              .forward(3)
//                .addDisplacementMarker(3, () -> {
//                            drive.turn(PI/2);
//                })
//                .forward(5)
//                .addDisplacementMarker(8, () -> {
////                            servoWork.armSpecimen();
////                            servoWork.armUpdate();
////                            servoWork.clawClosed();
////                            servoWork.armUp();
//                })
                .splineTo(new Vector2d(40,40),Math.toRadians(0))
//                .back(33)
//                .addDisplacementMarker(62, () -> {
//                            drive.turn(-PI/2);
////                            servoWork.linearUpSubmersible();
//                })
//                .forward(3)
//                .addDisplacementMarker(65, () -> {
////                            servoWork.linearDownALittleBit();
////                            servoWork.clawOpen();
////                            servoWork.linearDownFullFromSubmersible();
//                })
//                .back(24)
//                .addDisplacementMarker(89, () -> {
//                            drive.turn(PI/2);
//                })
//                .forward(33)
//                .addDisplacementMarker(122, () -> {
////                            servoWork.armSpecimen();
////                            servoWork.armUpdate();
////                            servoWork.clawClosed();
////                            servoWork.armUp();
//                })
//                .back(36)
//                .addDisplacementMarker(158, () -> {
//                            drive.turn(-PI/2);
//                })
//                .forward(21)
//                .addDisplacementMarker(179, () -> {
////                            servoWork.linearUpSubmersible();
//
//                })
//                .forward(3)
//                .addDisplacementMarker(182, () -> {
////                            servoWork.linearDownALittleBit();
////                            servoWork.clawOpen();
////                            servoWork.linearDownFullFromSubmersible();
//                })
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}

package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

@Autonomous
public class SampleSide extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoWork  servos = new servoWork();

        Trajectory sampleSide = drive.trajectoryBuilder(new Pose2d())
                .forward(3)
                .addDisplacementMarker(3, () -> {
                    drive.turn(PI/2);
                })
                .forward(5)
                .addDisplacementMarker(8, () -> {
//                            servoWork.armSpecimen();
//                            servoWork.armUpdate();
//                            servoWork.clawClosed();
//                            servoWork.armUp();
                })
                .strafeLeft(21)
                .back(33)
                .addDisplacementMarker(62, () -> {
                    drive.turn(-PI/2);
//                            servoWork.linearUpSubmersible();
                })
                .forward(3)
                .addDisplacementMarker(65, () -> {
//                            servoWork.linearDownALittleBit();
//                            servoWork.clawOpen();
//                            servoWork.linearDownFullFromSubmersible();
                })
                .back(24)
                .addDisplacementMarker(89, () -> {
                    drive.turn(PI/2);
                })
                .forward(33)
                .addDisplacementMarker(122, () -> {
//                            servoWork.armSpecimen();
//                            servoWork.armUpdate();
//                            servoWork.clawClosed();
//                            servoWork.armUp();
                })
                .back(36)
                .addDisplacementMarker(158, () -> {
                    drive.turn(-PI/2);
                })
                .forward(21)
                .addDisplacementMarker(179, () -> {
//                            servoWork.linearUpSubmersible();

                })
                .forward(3)
                .addDisplacementMarker(182, () -> {
//                            servoWork.linearDownALittleBit();
//                            servoWork.clawOpen();
//                            servoWork.linearDownFullFromSubmersible();
                })
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(sampleSide);

    }
}

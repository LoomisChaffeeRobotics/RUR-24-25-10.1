package com.example.meepmeeptesting2;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24 , -60, Math.toRadians(90)))
                        .forward(3)
                        .turn(-PI/2)
                        .addDisplacementMarker(3, () -> {
//                            drive.turn(PI/2);
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
                        .turn(PI/2)
                        .addDisplacementMarker(62, () -> {
//                            drive.turn(-PI/2);
//                            servoWork.linearUpSubmersible();
                        })
                        .forward(3)
                        .addDisplacementMarker(65, () -> {
//                            servoWork.linearDownALittleBit();
//                            servoWork.clawOpen();
//                            servoWork.linearDownFullFromSubmersible();
                        })
                        .back(24)
                        .turn(-PI/2)
                        .addDisplacementMarker(89, () -> {
//                            drive.turn(PI/2);
                        })
                        .forward(33)
                        .addDisplacementMarker(122, () -> {
//                            servoWork.armSpecimen();
//                            servoWork.armUpdate();
//                            servoWork.clawClosed();
//                            servoWork.armUp();
                        })
                        .back(36)
                        .turn(PI/2)
                        .addDisplacementMarker(158, () -> {
//                            drive.turn(-PI/2);
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
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


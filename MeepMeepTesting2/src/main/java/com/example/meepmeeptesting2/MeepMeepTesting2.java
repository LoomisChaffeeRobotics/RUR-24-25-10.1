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

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        .forward(24)
                        .addDisplacementMarker(24, () -> {
                            //servoWork.linearup();
                        })
                        .forward(2)
                        .addDisplacementMarker(26, () -> {
                            //servoWork.linearDown();
                            //servoork.clawopen();
                        })
                        .strafeLeft(30) //we appreciate not vandalizing the field
                        .back(10)
                        .strafeLeft(18)
                        .addDisplacementMarker(84, () -> {
                            //armdown,clawopen,close,armup
                        })
                        .back(10) //change to adapt
                        .turn(3*PI/4)
                        .strafeRight(3)
                        .addDisplacementMarker(97, () -> {

                        })
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
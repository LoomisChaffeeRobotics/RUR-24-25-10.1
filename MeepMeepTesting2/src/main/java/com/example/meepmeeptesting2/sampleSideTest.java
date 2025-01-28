package com.example.meepmeeptesting2;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static java.lang.Math.PI;




public class sampleSideTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double distance2Spec = 10d;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24 , -60, Math.toRadians(90)))
                        .forward(3)
                        .strafeLeft(25)
                        .forward(distance2Spec)
                        .addDisplacementMarker(() -> {

                        })
                        .turn(3*PI/4)
                        .forward(distance2Spec)
                        .addDisplacementMarker(() -> {

                        })
                        .turn()


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


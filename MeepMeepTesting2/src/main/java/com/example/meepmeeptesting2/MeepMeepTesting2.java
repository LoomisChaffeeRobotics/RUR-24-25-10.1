package com.example.meepmeeptesting2;

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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                        .strafeLeft(36) //we appreciate not vandalizing the field
                        .forward(33.3) //change to adapt
                        .strafeLeft(12)
                        .turn(Math.toRadians(180))
                        .addDisplacementMarker(72, () -> {
                            // something like Arm.movedown(), claw.open(), claw.close(), arm.moveup()
                        })
                        .forward(48)
                        .turn(Math.toRadians(-45))
                        .addDisplacementMarker(72, () -> {
                            // something like linearslide.moveup(), arm.movedown(), claw.open(), claw.close(), arm.moveup(), linearslide.movedown();
                            //use extender as needed
                        })
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
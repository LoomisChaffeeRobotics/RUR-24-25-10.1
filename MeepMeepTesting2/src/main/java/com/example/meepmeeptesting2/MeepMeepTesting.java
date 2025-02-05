package com.example.meepmeeptesting2;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static java.lang.Math.PI;




public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double distance2Spec = 10d;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(56.85543109255018, 56.85543109255018, 5.12, 3.6680923285425253, 13.46)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24 , -60, Math.toRadians(90)))
                        .forward(5)
                        .turn(-PI/2)
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(5, -36), Math.toRadians(90))
                        .turn(PI/2)
                        .forward(3)
                        .addDisplacementMarker(() -> {
//                            servos.linearUpSubmersible();
                            //servos.clawOpen();
                            //servos.linearDownFullFromSubermsible();
                        })
                        .back(3)
                        //We might make another trajectory right here in order to re-adjust the pose of the robot
                        .splineToConstantHeading(new Vector2d(24,-50),Math.toRadians(90))
                        .turn(-PI/2)
                        .strafeRight(5)
                        .forward(5)
                        .addDisplacementMarker(() -> {
                            //servos.armDown();
                            //servos.clawClosed();
                            //servos.armUp();
                        })
                        .splineToConstantHeading(new Vector2d(-5,-38), Math.toRadians(0))
                        .turn(PI/2)
                        .forward(5)
                        .addDisplacementMarker(() -> {
                            //servos.linearUpSubmersible();
                            //servos.clawOpen();
                            //servos.linearDownFullFromSubmersible();
                        })
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


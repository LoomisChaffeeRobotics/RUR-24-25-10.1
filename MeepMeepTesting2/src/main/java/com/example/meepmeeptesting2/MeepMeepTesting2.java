package com.example.meepmeeptesting2;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static java.lang.Math.PI;



public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(56.85543109255018, 56.85543109255018, 4.7, 3.6680923285425253, 13.46)


                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24 , -64.5, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(-48,-48), Math.toRadians(90))
                        .addDisplacementMarker(() -> {
                            //servoWork.armDown();
//                            servoWork.clawClosed();
                            //servoWork.armUp();
                        })
                        .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                        .turn(Math.toRadians(135))
                        .addDisplacementMarker(() -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })
                        .splineToConstantHeading(new Vector2d(-53,-47), Math.toRadians(90))
                        .turn(Math.toRadians(-135))
                        .addDisplacementMarker(() -> {
                            //servoWork.armDown();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                        })
                        .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                        .turn(Math.toRadians(135))
                        .addDisplacementMarker(() -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })
                        .splineToConstantHeading(new Vector2d(-56,-42), Math.toRadians(90))
                        .turn(Math.toRadians(-95))
                        .addDisplacementMarker(() -> {
                            //servoWork.armDown();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                        })
                        .splineToConstantHeading(new Vector2d(-53,-53), Math.toRadians(45))
                        .turn(Math.toRadians(95))
                        .addDisplacementMarker(() -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })
//
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


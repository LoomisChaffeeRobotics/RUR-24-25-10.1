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
                        .addDisplacementMarker(28+distance2Spec, () -> {
                            //servoWork.armDown();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                        })
                        .turn(3*PI/4)
                        .forward(distance2Spec)
                        .addDisplacementMarker(28+2*distance2Spec, () -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })
                        .back(distance2Spec)
                        .turn(-3*PI/4)
                        .strafeLeft(6)
                        .addDisplacementMarker(40 + 4*distance2Spec, () -> {
                            //servoWork.armDown();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                        })
                        .strafeRight(6)
                        .turn(3*PI/4)
                        .forward(distance2Spec)
                        .addDisplacementMarker(52+6*distance2Spec, () -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })
                        .back(distance2Spec)
                        .strafeRight(6)
                        .turn(-PI/2)
                        .forward(3)
                        .addDisplacementMarker(61 + 7*distance2Spec, () -> {
                            //servoWork.armDown();
                            //servoWork.clawClosed();
                            //servoWork.armUp();f
                        })
                        .back(3)
                        .turn(PI/2)
                        .strafeLeft(6)
                        .forward(distance2Spec)
                        .addDisplacementMarker(70+8*distance2Spec, () -> {
                            //servoWork.linearUpHigh();
                            //servoWork.arm45();
                            //servoWork.clawOpen();
                            //servoWork.clawClosed();
                            //servoWork.armUp();
                            //servoWork.linearDownFullFromHigh();
                        })

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


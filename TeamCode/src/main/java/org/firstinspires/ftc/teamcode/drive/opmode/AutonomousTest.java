package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.DoNothingWithCameraOn;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

@Config
@Autonomous
public class AutonomousTest extends OpMode {
    org.firstinspires.ftc.teamcode.servoWork servoWork;
//    DoNothingWithCameraOn cameraStuff;
    IMU imu;
    SampleMecanumDrive drive;
    DcMotor linearRight;
    DcMotor linearLeft;
    public double distance2Spec = 10d;

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);
        servoWork = new servoWork();
        servoWork.init(hardwareMap);
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void start() {
//       for (int i = 0; i < 15000; i++){
//           servoWork.armUpdate();
////           cameraStuff.cameraPoseUpdate();
//       }
       Pose2d specimenStartPose = new Pose2d(24,-64,Math.toRadians(-90));
        Trajectory specimenSide = drive.trajectoryBuilder(specimenStartPose)
                .strafeLeft(1)
                .build();
//                .addDisplacementMarker(3, () -> {
////                            drive.turn(PI/2);
//                })
////                .forward(5)
//                .addDisplacementMarker(8, () -> {
////                            servoWork.armSpecimen();
////                            servoWork.armUpdate();
////                            servoWork.clawClosed();
////                            servoWork.armUp();
//                })
//                .splineTo(new Vector2d(5,-33), Math.toRadians(-90))
//                .addDisplacementMarker(() -> {
//                            drive.turn(-PI/2);
////                            servoWork.linearUpSubmersible();
//                })
//                .forward(3)
//                .addDisplacementMarker(() -> {
////                            servoWork.linearDownALittleBit();
////                            servoWork.clawOpen();
////                            servoWork.linearDownFullFromSubmersible();
//                })
//                .splineTo(new Vector2d(24,-60), Math.toRadians(180))
//                .addDisplacementMarker( () -> {
////                            servoWork.armSpecimen();
////                            servoWork.armUpdate();
////                            servoWork.clawClosed();
////                            servoWork.armUp();
//                })
//                .splineTo(new Vector2d(-5,-33), Math.toRadians(90))
//                .addDisplacementMarker( () -> {
////                            servoWork.linearUpSubmersible();
//
//                })
//                .forward(3)
//                .addDisplacementMarker( () -> {
////                            servoWork.linearDownALittleBit();
////                            servoWork.clawOpen();
////                            servoWork.linearDownFullFromSubmersible();
//                })
        drive.followTrajectory(specimenSide);
//        Trajectory sampleSide = drive.trajectoryBuilder(new Pose2d())
//                .forward(3)
//                .strafeLeft(25)
//                .forward(distance2Spec)
//                .addDisplacementMarker(28+distance2Spec, () -> {
//                    //servoWork.armDown();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();
//                    drive.turn(3*PI/4);
//                })
//                .forward(distance2Spec)
//                .addDisplacementMarker(28+2*distance2Spec, () -> {
//                    //servoWork.linearUpHigh();
//                    //servoWork.arm45();
//                    //servoWork.clawOpen();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();
//                    //servoWork.linearDownFullFromHigh();
//                })
//                .back(distance2Spec)
//                .addDisplacementMarker(28+3*distance2Spec, () -> {
//
//                })
//                .strafeLeft(6)
//                .addDisplacementMarker(40 + 4*distance2Spec, () -> {
//                    //servoWork.armDown();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();
//                })
//                .strafeRight(6)
//                .addDisplacementMarker(46+4*distance2Spec, () -> {
//                    drive.turn(3*PI/4);
//                })
//                .forward(distance2Spec)
//                .addDisplacementMarker(52+6*distance2Spec, () -> {
//                    //servoWork.linearUpHigh();
//                    //servoWork.arm45();
//                    //servoWork.clawOpen();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();
//                    //servoWork.linearDownFullFromHigh();
//                })
//                .back(distance2Spec)
//                .strafeRight(6)
//                .addDisplacementMarker(58+7*distance2Spec, () -> {
//                    drive.turn(-PI/2);
//                })
//                .forward(3)
//                .addDisplacementMarker(61 + 7*distance2Spec, () -> {
//                    //servoWork.armDown();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();f
//                })
//                .back(3)
////                .turn(PI/2)
//                .strafeLeft(6)
//                .forward(distance2Spec)
//                .addDisplacementMarker(70+8*distance2Spec, () -> {
//                    //servoWork.linearUpHigh();
//                    //servoWork.arm45();
//                    //servoWork.clawOpen();
//                    //servoWork.clawClosed();
//                    //servoWork.armUp();
//                    //servoWork.linearDownFullFromHigh();
//                })
//
//                .build();
    }

    @Override
    public void loop() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servoWork.armUpdate();

//        Trajectory SpecimenHangTest = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(specimenLeft)
//                .forward(specimenForwardFirst) //will have to change based off robot size
//                .addDisplacementMarker(specimenLeft+specimenForwardFirst, () -> {
//                    servoWork.linearUpSubmersible();
//                })
//                .forward(specimenForwardSecond)
//                .addDisplacementMarker(specimenLeft+specimenForwardFirst+specimenForwardSecond, () -> {
//                    servoWork.linearDownALittleBit();
//                    servoWork.clawOpen();
//                })
//                .back(specimenForwardFirst + specimenForwardSecond)
//                .strafeRight(specimenRight)
//                .build();
//        Trajectory RightSideSamplesTest = drive.trajectoryBuilder(new Pose2d())
//                .strafeRight(35)
//                .forward(30)
//                .strafeRight(12)
//                .forward(10)
//                .addDisplacementMarker(113, () -> {
//                    servoWork.clawOpen();
//                    servoWork.armDown();
//                    servoWork.clawClosed();
//                    servoWork.armUp();
//                })
//                .back(65)
//                .forward(55)
//                .strafeRight(9)
//                .back(55)
//                .forward(55)
//                .strafeRight(4)
//                .back(55)
//                .build();
//
//        Trajectory LeftSideSamplesTest = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(30)
//                .back(10) // again, change based off how far the arm extends out
//                .strafeLeft(18)
//                .addDisplacementMarker(84, () -> {
//                    servoWork.clawOpen();
//                    servoWork.armDown();
//                    servoWork.clawClosed();
//                    servoWork.armUp();
//                })
//                .back(10) // change such that it goes 20 inches back total in auto3
//                .addDisplacementMarker(94, () -> {
//                    drive.turn(135); //test whether it goes 135 or -135 degrees
//                })
//                .strafeRight(3) //important
//                .addDisplacementMarker(97, () -> {
//                    servoWork.linearUpHigh();
//                    servoWork.arm45();
//                    servoWork.clawOpen();
//                    servoWork.armUp();
//                    servoWork.linearDownFullFromHigh();
//                })
//                .build();





//        drive.followTrajectory(Auto2);
//        drive.followTrajectory(Auto3);




    }

}

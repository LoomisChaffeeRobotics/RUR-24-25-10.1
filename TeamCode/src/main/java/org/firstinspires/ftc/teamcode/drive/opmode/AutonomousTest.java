package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

public class AutonomousTest extends OpMode {
    servoWork servoWork;
    IMU imu;
    SampleMecanumDrive drive;
    DcMotor linearRight;
    DcMotor linearLeft;


    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);
        servoWork = new servoWork();
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void loop() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory Auto1 = drive.trajectoryBuilder(new Pose2d())
                .forward(24)
                .addDisplacementMarker(24, () -> {
                    //servoWork.linearup();
                })
                .forward(2)
                .addDisplacementMarker(26, () -> {
                    //servoWork.linearDown();
                    servoWork.clawOpen();
                })
                .build();
        Trajectory Auto2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(35)
                .forward(30)
                .strafeRight(12)
                .back(55)
                .forward(55)
                .strafeRight(9)
                .back(55)
                .forward(55)
                .strafeRight(4)
                .back(55)
                .build();

        Trajectory Auto3 = drive.trajectoryBuilder(new Pose2d())
                //if we start on left, picking up and depositing one sample
                .strafeLeft(36) //we appreciate not vandalizing the field
                .back(33.3) //change to adapt
                .addDisplacementMarker(81.3, () -> {
                    // something like Arm.movedown(), claw.open(), claw.close(), arm.moveup()
                    drive.turn(Math.toRadians(-135));
                    servoWork.armDown();
                    servoWork.clawOpen();
                    servoWork.clawClosed();
                    servoWork.armUp();
                })
                .back(48)
                .build();



        drive.followTrajectory(Auto1);
        drive.followTrajectory(Auto2);
//        drive.followTrajectory(Auto2);
        drive.followTrajectory(Auto3);
        //motor1/2/3.turn(something)




    }

}

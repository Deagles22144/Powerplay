package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="autoCar", group="Robot")
public class autoCar extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();

        }

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 25), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-35, 11, Math.toRadians(135)), Math.toRadians(-90))
                .build();

        TrajectorySequence FirstBackDrive = drive.trajectorySequenceBuilder(traj.end())
                .back(3, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(FirstBackDrive.end())
                .setTangent(Math.toRadians(105))
                .splineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,15, Math.toRadians(180)),Math.toRadians(180))
                .forward(5.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-45, 15, Math.toRadians(180)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-37, 12 , Math.toRadians(135)),Math.toRadians(-50))
                .build();

        TrajectorySequence SecondBackDrive = drive.trajectorySequenceBuilder(traj2.end())
                .back(5, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        waitForStart();


        drive.followTrajectorySequence(traj);
        elevatorHigh();
        tilt.setPosition(tiltAuto);

        sleep(2500);
        drive.followTrajectorySequence(FirstBackDrive);
        telemetry.addLine("elevator pos: " + elevator1.getCurrentPosition());

        if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 100)) {
            claw.setPosition(clawOpen);
            sleep(750);
        }
        elevatorGround();

        while (elevator0.getCurrentPosition() >= elevatorMiddlePos / 2) {
            armPos(0.05);
        }
        armPos(0.01);
        tilt.setPosition(tiltGround);
        sleep(600);


        for (int i = 0; i < 5; i++) {
            elevatorAuto(cones[i]);

            drive.followTrajectorySequence(traj1);


            claw.setPosition(clawClose);
            tilt.setPosition(tiltLow);
            sleep(1000);
            elevatorAfterColloctAuto();

            drive.followTrajectorySequence(traj2);

            elevatorHigh();
            tilt.setPosition(tiltAuto);

            sleep(2500);
            drive.followTrajectorySequence(SecondBackDrive);
            //drive.followTrajectorySequence(SecondBackDrive);
            telemetry.addLine("elevator pos: " + elevator1.getCurrentPosition());
            if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 10))
            {
                claw.setPosition(clawOpen);
                sleep(750);
            }
            elevatorGround();
            while (elevator0.getCurrentPosition() >= elevatorMiddlePos / 2) {
                armPos(0.05);
            }
            armPos(0.01);
            tilt.setPosition(tiltGround);
            sleep(600);

        }

            elevatorGround();

            if (tagOfInterest == null || tagOfInterest.id == parkLeft) {
                //left code

            } else if (tagOfInterest.id == parkMiddle) {
                //middle code
            } else if (tagOfInterest.id == parkRight) {
                //right code

            }
        }


    }




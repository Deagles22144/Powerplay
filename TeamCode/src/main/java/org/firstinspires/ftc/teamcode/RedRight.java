package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous (name="RedRight", group="AutoRight")
public class RedRight extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();

        }

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 25), Math.toRadians(-90))
                .addDisplacementMarker(2,() -> {
                    armPos(0.23);
                })
                .addDisplacementMarker(10,() -> {
                    // tiltPos(tiltHigh);
                    elevatorHighAuto();
                })
                .splineToSplineHeading(new Pose2d(-33, 10, Math.toRadians(135)), Math.toRadians(-70))
                //.waitSeconds(1)
                .build();


        TrajectorySequence FirstDriveToCones = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(120))
                .splineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,15, Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(8,() -> {
                    elevatorAuto(cones[0]);
                  //  tiltPos(tiltGround);
                })
                .forward(3.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence ConeUnload = drive.trajectorySequenceBuilder(new Pose2d(-63,15, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-45, 15, Math.toRadians(180)),Math.toRadians(0))
                .addDisplacementMarker(2,() -> {
                    //tiltPos(tiltHigh);
                    elevatorHighAuto();
                })

                .splineToSplineHeading(new Pose2d(-34, 12 , Math.toRadians(135)),Math.toRadians(-30),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .build();

        TrajectorySequence ConeLoad = drive.trajectorySequenceBuilder(ConeUnload.end())//FirstBackDrive.end())
                .setTangent(Math.toRadians(120))
                .splineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,15, Math.toRadians(180)),Math.toRadians(180))
//                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence forwardDrive = drive.trajectorySequenceBuilder(ConeLoad.end())
                .forward(3.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(120)
                .lineToLinearHeading(new Pose2d(-35,24,Math.toRadians(90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-35,25,Math.toRadians(115)),Math.toRadians(115))
                .splineToSplineHeading( new Pose2d(-64,35,Math.toRadians(180)),Math.toRadians(180))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-28,13,Math.toRadians(180)),Math.toRadians(0))
                .splineToSplineHeading( new Pose2d(-10,25,Math.toRadians(-90)),Math.toRadians(90))
                .build();


        waitForStart();

        //rasing the arm so that the cone wont be on the floor.

        drive.followTrajectorySequence(Preload);
//        elevatorHigh();
        armPos(armHighAuto);
        sleep(2500);

        if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20) || elevator0.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
        {
            claw.setPosition(clawOpen);
            sleep(750);
            isGround = true;
            elevatorGround();
        }
        /*if (elevator0.getCurrentPosition() <= elevatorMiddlePos/2 && isGround) {
            armPos(armGround);
            tilt.setPosition(tiltGround);
            isGround = false;
        }

         */
        while (elevator0.getCurrentPosition() >= 250)
        {
            armPos(armLow);
        }

        armPos(armGround);
       // tiltPos(tiltGround);
        sleep(600);

        drive.followTrajectorySequence(FirstDriveToCones);


        for (int i = 1; i < 2; i++)
        {

            drive.setPoseEstimate(new Pose2d(-63,drive.getPoseEstimate().getY()/*15*/, drive.getPoseEstimate().getHeading() /*Math.toRadians(180)*/));

            claw.setPosition(clawClose);
           // tiltPos(tiltGround);
            sleep(1000);
            elevatorAfterColloctAuto();

            drive.followTrajectorySequence(ConeUnload);
//            elevatorHigh();
            armPos(armHighAuto);
            sleep(2000);

/*
            elevatorHigh();
            tilt.setPosition(tiltAuto);

            sleep(2500);
            */

            // drive.followTrajectorySequence(SecondBackDrive);
            //drive.followTrajectorySequence(SecondBackDrive);

            telemetry.addLine("elevator pos: " + elevator1.getCurrentPosition());
            if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
            {
                claw.setPosition(clawOpen);
                sleep(750);
                isGround = true;
            }

            //elevatorAuto(cones[i]);
            elevatorGround();
            while (elevator0.getCurrentPosition() >= 250)
            {
                armPos(armLow);
            }

            armPos(armGround);
           // tiltPos(tiltGround);
            sleep(600);

            if(i < 1)
            {
                drive.followTrajectorySequence(ConeLoad);
                elevatorAuto(cones[i]);
                drive.followTrajectorySequence(forwardDrive);
            }
        }

        elevatorGround();

        if (tagOfInterest == null || tagOfInterest.id == parkLeft)
        {
            //left code
            armPos(armGround);
            drive.followTrajectorySequence(ParkLeft);

        } else if (tagOfInterest.id == parkMiddle)
        {
            //middle code
            armPos(armGround);
            drive.followTrajectorySequence(ParkMid);

        } else if (tagOfInterest.id == parkRight)
        {
            //right code
            armPos(armGround);
            drive.followTrajectorySequence(ParkRight);

        }
    }


}




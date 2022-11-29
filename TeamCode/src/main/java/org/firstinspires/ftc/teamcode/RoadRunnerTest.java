package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous (name="RoadRunnerTest", group="Robot")
public class RoadRunnerTest extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()){
            QrScan();

        }


        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20, 10),Math.toRadians(0))
                .build();

        drive.followTrajectory(traj1);


        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj1.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build());*/


       /* if (tagOfInterest == null || tagOfInterest.id == left) {
            //left code
            encoderDriveP(1,-10,-10,-10,-10,0.019,2);
            RotateP(-90,1,2,0.019);
            encoderDriveP(1,55,55,55,55,0.019,2);
            RotateP(-180,1,2,0.019);
            encoderDriveP(1,60,60,60,60,0.019,2);
            RotateP(-180,1,2,0.019);
        } else if (tagOfInterest.id == middle) {
            //middle code
            encoderDriveP(1, -60, -60, -60, -60, 0.019, 1);
            RotateP(-180,1,2,0.019);
        }
        else if (tagOfInterest.id == right) {
            //right code
            encoderDriveP(1, -10, -10, -10, -10, 0.019, 2);
            RotateP(90, 1, 2, 0.019);
            encoderDriveP(1, 50, 50, 50, 50, 0.019, 2);
            RotateP(0, 1, 2, 0.019);
            encoderDriveP(1, -65, -65, -65, -65, 0.019, 2);
            RotateP(-180,1,2,0.019);
        }*/
    }
}

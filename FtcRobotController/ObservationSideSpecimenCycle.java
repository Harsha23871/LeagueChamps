public class ObservationSideSpecimenCycle {package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

    @Autonomous(group = "a", name="ObservationSideSpecimen")
    public class ObservationSideSpecimen extends LinearOpMode {

        @Override

// 12 -5

        public void runOpMode() {
            waitForStart();
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(11,  -61, Math.toRadians(270));
            drive.setPoseEstimate(startPose);
            TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270)))
                    .splineToConstantHeading(new Vector2d(0.00, -31.81), Math.toRadians(34.39))
                    .build();



            drive.followTrajectorySequence(trajectory0);









        /*TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/








               /*
                TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/










                /*.splineTo(new Vector2d(-49.46, -26.18), Math.toRadians(111.36))



     //   Trajectory myTrajectory = drive.trajectoryBuilder(Traj1.end())
     //           .splineTo(new Vector2d(48 , -48), 0)
      //          .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectory7);

      /*  Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();*/

            //   drive.followTrajectory(myTrajectory);
//splineTo(new Vector2d(x1, y1), heading)
            //       .splineTo(new Vector2d(x2, y2), heading)
            //       .build();

// -12 -48

       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62,Math.toRadians(270));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-62))
//                .turn(Math.toRadians(45)) // Turns 45 degrees counter-clockwise
                .build();


        waitForStart();
        drive.followTrajectorySequence(middleSpike);//might need to change*/


        }   }

}

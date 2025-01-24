package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="Test1 Cycle")
public class LG2AutoOBSV extends LinearOpMode {
    public DcMotor elevator = null;
    public Servo claw = null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw =  hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(1);


//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position


        //  for (int i = 0; i < 5; i++) {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);


        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(34.39),

                 SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                ) // important
                .build();


        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end()) // .plus(new Pose2d(0, 0, Math.toRadians(180)))
              //  .splineToConstantHeading(new Vector2d(47, -64), Math.toRadians(34.39),
                .lineToSplineHeading(new Pose2d(47, -54, Math.toRadians(91))) // 180 or 190 90
            //     SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
             //   SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

             //   )
                .build();
//  .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(new Pose2d(47, -64, Math.toRadians(270))) //90 270
                .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(34.39), // -62 55.61 124.39

               SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end()) // .plus(new Pose2d(0, 0, Math.toRadians(90)))
             //   .splineToConstantHeading(new Vector2d(47, -64), Math.toRadians(34.39), // -62
                .lineToSplineHeading(new Pose2d(47, -54, Math.toRadians(91))) // 180 190 90
//                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)


              //  )
                .build();


//        TrajectorySequence FrontALittle = drive.trajectorySequenceBuilder(trajectory1.end()) // .plus(new Pose2d(0, 0, Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(47, -62), Math.toRadians(91), // -62
//
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        // important
//                )
//                .build();

//        TrajectorySequence BackALittle = drive.trajectorySequenceBuilder(FrontALittle.end()) // .plus(new Pose2d(0, 0, Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(47, -54), Math.toRadians(91), // -62
//
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        // important
//                )
//                .build();





        waitForStart();




        // add claw + elevator
           elevator.setTargetPosition(1900);
           elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           elevator.setPower(0.6);


        drive.followTrajectorySequence(trajectory0);
            elevator.setTargetPosition(0); //  1500
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.8);



        // add claw + elevator
        drive.followTrajectorySequence(trajectory1);
        claw.setPosition(0.7);
      //  drive.followTrajectorySequence(FrontALittle);
      //  drive.followTrajectorySequence(BackALittle);

        claw.setPosition(1);
    //    sleep(1000);


        // 0.7 open
        // 1 is close
        for (int i = 0; i < 1; i++) {
          //  drive.turn(Math.toRadians(180)); // maybe change
          //  claw.setPosition(1);
            sleep(1000); // originally 2000


            elevator.setTargetPosition(1900);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.6);
            drive.followTrajectorySequence(trajectory3);
//
            elevator.setTargetPosition(1500);
           elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.9);

            sleep(1000);

            claw.setPosition(0.7);
            elevator.setTargetPosition(0);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.9);

            drive.followTrajectorySequence(trajectory4);
            claw.setPosition(0.7);   // 143 - 146 lines need change
        //    drive.followTrajectorySequence(BackALittle);
            claw.setPosition(1);
        //    drive.turn(Math.toRadians(180));
         //   drive.followTrajectorySequence(BackALittle);


        }

    //    drive.followTrajectorySequence(trajectory2);


        // 35.81,-37.15 start pos
        if (isStopRequested()) return;
        //robot.hwMap();

//        sleep(2000);


        //   }






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




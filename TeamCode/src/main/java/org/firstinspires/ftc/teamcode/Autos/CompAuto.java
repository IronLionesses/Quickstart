//package org.firstinspires.ftc.teamcode.Autos;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static org.opencv.imgproc.Imgproc.line;

//import static java.util.concurrent.locks.LockSupport.park;

//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.robocol.Command;

//import org.firstinspires.ftc.teamcode.pedro.Constants;

//public class CompAuto {package
  //  org.firstinspires.ftc.teamcode.Autos;
//}

 //import com.pedropathing.api.PoseFactory;
 //import com.pedropathing.follower.Follower;
 //import com.pedropathing.ivy.Command;
 //import com.pedropathing.ivy.Scheduler;
 //import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
 //import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;

//import static com.pedropathing.api.Paths.line;
 //import static com.pedropathing.ivy.Scheduler.schedule;
 //import static com.pedropathing.ivy.commands.Commands.waitMs;
 //import static com.pedropathing.ivy.groups.Groups.sequential;
 //import static com.pedropathing.ivy.pedro.PedroCommands.follow;

//import static telemetry;
//import static park;


//import org.firstinspires.ftc.teamcode.pedro.Constants;


//@Autonomous(name = "CompAuto") {
  //  class ExampleAuto extends OpMode


    //private Follower follower;
    //private DcMotor transfer;
    //private DcMotor intake;
    //private DcMotor shooter;
    //private static final PoseFactory poseFactory = PoseFactory.degrees();


    //private static final Pose start = poseFactory.of(9.3663, 8.2665, 90);
    //private static final Pose path1 = poseFactory.of(22.4237, 41.3296, 180);
    //private static final Pose point2Start = poseFactory.of(22.4237, 41.3296, 0);
    //private static final Pose point2 = poseFactory.of(56.725, 24.6412, 0);
    //private static final Pose point3 = poseFactory.of(71.2524, 11.6902, 138.2831);
    //private static final Pose point4 = poseFactory.of(94.1092, 13.5697, -175.2993);
    //private static final Pose point5 = poseFactory.of(57.822, 24.6186, -16.9348);
    //private static final Pose park = poseFactory.of(10.6469, 105.0659, -59.6122);


    //private static Path startToScore; {
     //   return line(start, path1)
      //          .linear(start, path1);


      //  private static Path path2; {
      //      return line(point2Start, point2)
      //              .linear(point2Start, point2);

      //  }

       // private static Path path3; {
        //    return line(point2, point3)
        //            .linear(point2, point3);
       // }

       // private static Path path4; {
        //    return line(point3, point4)
          //          .linear(point3, point4);

        //}

        //private static Path path5; {
          //  return line(point4, point5)
            //        .linear(point4, point5);

       // }

       // private static Path park; {
         //   return line(point4, park)
           //         .linear(point4, ExampleAuto.park);
        //}

        //private static Command mainAuto (Follower follower){
        //    return sequential(
          //          follow(follower, startToScore()),
            //        waitMs(1000),
              //      follow(follower, path2()),
              //      waitMs(1000),
              //      follow(follower, path3()),
              //      waitMs(1000),
                //    follow(follower, path4()),
                //    waitMs(1000),
                 //   follow(follower, path5()),
                  //  waitMs(1000),
                 //   follow(follower, park())
          //  );
    //    }

       // @Override
       // public void init () {
         //   Scheduler.reset();
           // follower = Constants.create(hardwareMap);

            //follower.setPose(start);
            //follower.update();
        //}

        //@Override
        //public void start () {
          //  schedule(mainAuto(follower));
        //}


        //@Override
        //public void loop () {
          //  follower.update();


            //Scheduler.execute();

            //telemetry.addData("X", follower.pose().x());
            //telemetry.addData("Y", follower.pose().y());
            //telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            //telemetry.addData("Follower Mode", follower.mode());
            //telemetry.update();
        //}


    //}

//}

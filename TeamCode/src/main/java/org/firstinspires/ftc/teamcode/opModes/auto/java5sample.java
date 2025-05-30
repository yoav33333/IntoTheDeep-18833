//package org.firstinspires.ftc.teamcode.opModes.auto;
//
//import static org.firstinspires.ftc.teamcode.subsystems.deposit.getArmIn;
//import static org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.commands.extendoCommand;
//import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
//import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.deposit;
//import org.firstinspires.ftc.teamcode.subsystems.linearSlides;
//import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking;
//import org.firstinspires.ftc.teamcode.commands.util.WaitUntil;
//
//import dev.frozenmilk.mercurial.commands.groups.Parallel;
//import dev.frozenmilk.mercurial.commands.groups.Sequential;
//import dev.frozenmilk.mercurial.commands.util.Wait;
//@Config
//@Autonomous
//public class java5sample extends AutoBaseJava {
//    public java5sample() {super(Side.basket);}
//
//    public Pose startPose = this.startingPoseBasket;
//    public Pose basketPose1 = new Pose(-58, 55, Math.toRadians(-45));
//    public Pose basketPose2 = new Pose(-56.9, 56.7, Math.toRadians(-45));
//    public Pose basketPose3 = new Pose(-56.9, 57.3, Math.toRadians(-45));
//    public Pose basketPose4 = new Pose(-55.5, 58.3, Math.toRadians(-45));
//    public Pose basketPose5 = new Pose(-55.9, 56.1, Math.toRadians(-45));
//    public Pose pickup1Pose = new Pose(-54.0, 48.8, 0);
//    public Pose pickup2Pose = new Pose(-54.9, 58.1, Math.toRadians(0));
//    public Pose pickup3Pose = new Pose(-53.4, 60, Math.toRadians(21));
//    public Pose pickup4Pose = new Pose(-58.7, 30, Math.toRadians(270));
//    public Pose parkPose = new Pose(-4, 19, Math.toRadians(90));
//    public Pose parkControl = new Pose(-9, 52, Math.toRadians(0));
//
//    static PathChain scorePreload;
//    static PathChain scorePickup1;
//    static PathChain scorePickup2;
//    static PathChain scorePickup3;
//    static PathChain scorePickup4;
//    static PathChain pickup1;
//    static PathChain pickup2;
//    static PathChain pickup3;
//    static PathChain pickup4;
//    static PathChain park;
//    @Override
//    public void myInit() {
//
//        scorePreload = makeLinePath(startPose, basketPose1);
//        pickup1 = makeLinePath(basketPose1, pickup1Pose);
//        scorePickup1 = makeLinePath(pickup1Pose, basketPose2);
//        pickup2 = makeLinePath(basketPose2, pickup2Pose);
//        scorePickup2 = makeLinePath(pickup2Pose, basketPose3);
//        pickup3 = makeLinePath(basketPose3, pickup3Pose);
//        scorePickup3 = makeLinePath(pickup3Pose, basketPose4);
//        pickup4 = makeSpinHalfWayPath(basketPose4, pickup4Pose);
//        scorePickup4 = makeSpinHalfWayPath(pickup4Pose, basketPose5);
//        park = makeCurvePath(basketPose5, parkControl, parkPose);
//
////        clawSubsystem.getClawRotationServo().setPosition(0.5);
//        clawSubsystem.getClawServo().setPosition(0.0);
//        deposit.armOutHalf();
//        deposit.closeClawRaw();
//
//    }
//
//
//    @Override
//    public void myStart() {
//
//        linearSlides.setPose(0);
////        linearSlides.getRunToPosition().cancel();
////        linearSlides.getRunToPosition().schedule();
//        new Sequential(
//                new RunNonBlocking(linearSlides.getRunToPosition()),
//
//                extendoCommand.getExtendoReset(),
//            linearSlides.getGoToHighBasket(),
//            new RunNonBlocking(
//                    new Sequential(
//                        new WaitUntil(()->(linearSlides.target>500 && linearSlides.target-10000<getPose())),
//                        deposit.getArmOut()
//                    )),
//            new WaitUntil(() -> getPose()>20000),
//            followPath(scorePreload),
//            new WaitUntil(() -> linearSlides.target-2500<getPose()),
//            deposit.getRelease(),
//            new Wait(0.1),
////            new Parallel(
//                followPath(pickup1),
//                new WaitUntil(()->follower.getCurrentTValue()>0.5),
//                extendoCommand.getExtendoOpenCommandAuto(),
////            ),
//            new Wait(0.35),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            new WaitUntil(()->linearSlides.getPose()<3000),
//            extendoCommand.getExtendoCloseCommandAuto(),
//            linearSlides.getGoToHighBasket(),
//            new WaitUntil(() -> getPose()>20000),
//            followPath(scorePickup1),
//            new WaitUntil(() -> linearSlides.target-2500<getPose()),
//            deposit.getRelease(),
//            new Wait(0.1),
////            new Parallel(
//                followPath(pickup2),
//                new WaitUntil(()->follower.getCurrentTValue()>0.5),
//                extendoCommand.getExtendoOpenCommandAuto(),
////            ),
//            new Wait(0.35),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            new WaitUntil(()->linearSlides.getPose()<2000),
//            extendoCommand.getExtendoCloseCommandAuto(),
//            linearSlides.getGoToHighBasket(),
//            new WaitUntil(() -> getPose()>20000),
//            followPath(scorePickup2),
//            new WaitUntil(() -> linearSlides.target-3000<getPose()),
//            deposit.getRelease(),
//            new Wait(0.1),
////            new Parallel(
//                followPath(pickup3),
//                new WaitUntil(()->follower.getCurrentTValue()>0.5),
//                extendoCommand.getExtendoOpenCommandAuto(),
////            ),
//            new Wait(0.3),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            new WaitUntil(()->linearSlides.getPose()<2000),
//            extendoCommand.getExtendoCloseCommandAuto(),
//            linearSlides.getGoToHighBasket(),
//            new WaitUntil(() -> getPose()>20000),
//            followPath(scorePickup3),
//                new WaitUntil(() -> linearSlides.target-3000<getPose()),
//            deposit.getRelease(),
//            new Wait(0.1),
////            new Parallel(
//                followPath(pickup4),
//                new WaitUntil(()->follower.getCurrentTValue()>0.95),
//                extendoCommand.getExtendoOpenCommandAuto(),
////            ),
//            new Wait(0.3),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            new WaitUntil(()->linearSlides.getPose()<3000),
//            extendoCommand.getExtendoCloseCommandAuto(),
//            linearSlides.getGoToHighBasket(),
//            new WaitUntil(() -> getPose()>20000),
//            followPath(scorePickup4),
//                new WaitUntil(() -> linearSlides.target-2500<getPose()),
//            deposit.getRelease(),
//            new Wait(0.1),
//            new Parallel(
//                followPath(park),
//                new Sequential(
//                    new WaitUntil(()->follower.getCurrentTValue()>0.3),
//                    linearSlides.getTouchBar(),
//                    getArmIn()
//                )
//            ),
//
//            finishAuto
//        ).schedule();
//    }
//    @Override
//    public void myStop() {
//        follower.breakFollowing();
//        runFollower.cancel();
//
////        linearSlides.getRunToPosition().cancel();
//        linearSlides.setStartingPose(getPose());
////        followerSubsystem.setStartingPose(follower.getPose());
//    }
//}

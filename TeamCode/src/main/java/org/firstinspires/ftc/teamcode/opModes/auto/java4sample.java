package org.firstinspires.ftc.teamcode.opModes.auto;



import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getAvoidBasket;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveSemiToWall;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveToChamber;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveToDeposit;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveToSlam;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveToWall;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getOpenExtension;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getCloseDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getOpenDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getCloseIntakeClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getResetAngleClaw;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getEnablePID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToHighBasket;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToWall;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getCloseCommand;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getOpenCommandAuto;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getReset;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getV4bWall;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.util.InstantCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;

import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking;
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Config
@Autonomous
public class java4sample extends AutoBaseJava {
    public java4sample() {super(Side.basket);}

    public Pose startPose = this.startingPoseBasket;
    public Pose basketPose1 = new Pose(-57.9, 57, Math.toRadians(-45));
    public Pose basketPose2 = new Pose(-56.9, 54.7, Math.toRadians(-45));
    public Pose basketPose3 = new Pose(-56.9, 55.3, Math.toRadians(-45));
    public Pose basketPose4 = new Pose(-55.5, 56.3, Math.toRadians(-45));
//    public Pose basketPose5 = new Pose(-55.9, 56.1, Math.toRadians(-45));
    public Pose pickup1Pose = new Pose(-50.0, 49.1, 0);
    public Pose pickup2Pose = new Pose(-50.2, 59.7, Math.toRadians(0));
    public Pose pickup3Pose = new Pose(-49.1, 60.6, Math.toRadians(25));
//    public Pose pickup4Pose = new Pose(-58.7, 30, Math.toRadians(270));
    public Pose parkPose = new Pose(-4, 19, Math.toRadians(90));
    public Pose parkControl = new Pose(-9, 52, Math.toRadians(0));

    static PathChain scorePreload;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
//    static PathChain scorePickup4;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
//    static PathChain pickup4;
    static PathChain park;
    @Override
    public void myInit() {

        scorePreload = makeLinePath(startPose, basketPose1);
        pickup1 = makeLinePath(basketPose1, pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, basketPose2);
        pickup2 = makeLinePath(basketPose2, pickup2Pose);
        scorePickup2 = makeLinePath(pickup2Pose, basketPose3);
        pickup3 = makeLinePath(basketPose3, pickup3Pose);
        scorePickup3 = makeLinePath(pickup3Pose, basketPose4);
//        pickup4 = makeSpinHalfWayPath(basketPose4, pickup4Pose);
//        scorePickup4 = makeSpinHalfWayPath(pickup4Pose, basketPose5);
        park = makeCurvePath(basketPose4, parkControl, parkPose);
        new Parallel(
                getResetAngleClaw(),
                getCloseDepositClaw(),
                new Sequential(
//                        getOpenExtension(),
                        getMoveSemiToWall(),
                        getReset(),
                        new Wait(0.3),
                        getV4bWall(),
                        getGoToWall(),
                        new Wait(0.5),
                        getCloseIntakeClaw()
                )
        ).schedule();



    }


    @Override
    public void myStart() {


        new Sequential(
            new Parallel(new RunNonBlocking(getAvoidBasket()),
            getEnablePID(),
            getReset(),
            getGoToHighBasket(),
            new RunNonBlocking(
                    new Sequential(
                        new WaitUntil(()->(targetPosition>500 && targetPosition-10000<getPose())),
                        getMoveToDeposit()
                    )),
            new WaitUntil(() -> getPose()>1000)),
            followPath(scorePreload),
            new WaitUntil(() -> targetPosition-3000<getPose()),
            new Wait(0.1),
            getOpenDepositClaw(),
            new Wait(0.1),
            followPath(pickup1),
            new WaitUntil(()->follower.getCurrentTValue()>0.5),
            getOpenCommandAuto(),
            new Wait(0.2),
            getCloseIntakeClaw(),
            new Wait(0.1),
            new WaitUntil(()->getPose()<targetPosition + 3000),
            getCloseCommand(),
            getGoToHighBasket(),
            followPath(scorePickup1),
            new WaitUntil(() -> targetPosition-2500<getPose()),
            new Wait(0.2),
            getOpenDepositClaw(),
            new Wait(0.1),
            followPath(pickup2),
            new WaitUntil(()->follower.getCurrentTValue()>0.5),
            getOpenCommandAuto(),
            new Wait(0.2),
            getCloseIntakeClaw(),
            new Wait(0.1),
            new WaitUntil(()->getPose()<targetPosition + 3000),
            getCloseCommand(),
            getGoToHighBasket(),
            followPath(scorePickup2),
            new WaitUntil(() -> targetPosition-3000<getPose()),
            new Wait(0.1),
            getOpenDepositClaw(),
            new Wait(0.10),
            followPath(pickup3),
            new WaitUntil(()->follower.getCurrentTValue()>0.5),
            getOpenCommandAuto(),
            new Wait(0.15),
            getCloseIntakeClaw(),
            new Wait(0.05),
            getCloseCommand(),
            getGoToHighBasket(),
            followPath(scorePickup3),
            new WaitUntil(() -> targetPosition-3000<getPose()),
            new Wait(0.2),
            getOpenDepositClaw(),
            new Wait(0.1),
            new Parallel(
                followPath(park),
                new Sequential(
                    new WaitUntil(()->follower.getCurrentTValue()>0.3)
                )
            ),

            finishAuto,
            new InstantCommand(()->targetPosition-=9000),
            getMoveToSlam()
        ).schedule();
    }
    @Override
    public void myStop() {
        follower.breakFollowing();
        runFollower.cancel();
        finishAuto.schedule();
        LiftVariables.startingPose = getPose();
        DriveVariables.startingAngle = -follower.getPose().getHeading();
//        LiftVariables.startingPose = getPose();
    }
}

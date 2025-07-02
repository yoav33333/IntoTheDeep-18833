package org.firstinspires.ftc.teamcode.opModes.auto;


import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getAvoidBasket;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getMoveToWall;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getOpenExtension;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getCloseDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getCloseWhenSampleInPlace;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getOpenDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getCloseIntakeClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getResetAngleClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getTurnLeft;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getCloseSlides;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getEnablePID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToHighChamber;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToHighChamberUp;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToWall;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getChamberSeq;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getChamberSeqNoDelay;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getCloseNoTransfer;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getOpenPushCommand;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getReset;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getUp;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getWallSeq;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getWallSeqSimple;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getArmUp;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getExtendoPush;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getV4bWall;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables;
import org.firstinspires.ftc.teamcode.subsystems.robot.BulkReads;
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking;
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
@Config
@Autonomous
public class java5spec extends AutoBaseJava {
    public java5spec() {super(Side.chamber);}
    public static double offset = -6.9;
    public MultipleTelemetry tele = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    @Override
    public void myLoop(){
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    @Override
    public void myInitLoop() {
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    Pose startPose = this.startingPoseChamber;
    Pose chamberPose1 = new Pose(-36.8, -3, Math.toRadians(180));
    Pose chamberPose2 = new Pose(-32.0+offset, 2-7, Math.toRadians(180));
    Pose chamberPose3 = new Pose(-32.0+offset, 2-7, Math.toRadians(180));
    Pose chamberPose4 = new Pose(-32.0+offset, 2-7, Math.toRadians(180));
    Pose chamberPose5 = new Pose(-32.0+offset, 2-7, Math.toRadians(180));
    Pose dragPose1 = new Pose(-35, -28.0, Math.toRadians(-48));
    Pose dragPose2 = new Pose(-34, -38.0, Math.toRadians(-60));
    Pose dragPose3 = new Pose(-34, -46.7, Math.toRadians(-60));
    Pose dragPoseTurn1 = new Pose(-40.2, -26, Math.toRadians(-65-75));
    Pose dragPoseTurn2 = new Pose(-40.3, -37.2, Math.toRadians(-65-75));
    Pose dragPoseTurn3 = new Pose(-45, -47.5, Math.toRadians(-65-65));
    Pose pickup1Pose = new Pose(-57-0.6, -39, Math.toRadians(180));
    Pose pickup2Pose = new Pose(-57-0.6, -37, Math.toRadians(180));
    Pose pickup3Pose = new Pose(-57-0.6, -37, Math.toRadians(180));
    Pose pickup4Pose = new Pose(-57-0.6, -37, Math.toRadians(180));
//    public Pose pickup12Pose = new Pose(-59, -40, Math.toRadians(180));
    public Pose parkPose = new Pose(-47, -31, Math.toRadians(180+40));

    static PathChain scorePreload;
    static PathChain getToDrag1;
    static PathChain turnDrag1;
    static PathChain getToDrag2;
    static PathChain turnDrag2;
    static PathChain getToDrag3;
    static PathChain turnDrag3;
    static PathChain specialHPIntake;
//    static PathChain specialHPIntake1;
//    static PathChain specialHPIntake1score;
    static PathChain specialHPIntake2;
    static PathChain specialHPIntake2score;
    static PathChain dragToHP;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
    static PathChain scorePickup4;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
    static PathChain pickup4;
    static PathChain park;
    public static double power = 1.0;
    @Override
    public void myInit() {

        scorePreload = makeLinePath(startPose, chamberPose1);
        getToDrag1 = makeLinePath(chamberPose1, dragPose1);
        turnDrag1 = makeLinePath(dragPose1, dragPoseTurn1);
        getToDrag2 = makeLinePath(dragPoseTurn1, dragPose2);
        turnDrag2 = makeLinePath(dragPose2, dragPoseTurn2);
        getToDrag3 = makeLinePath(dragPoseTurn2, dragPose3);
        turnDrag3 = makeLinePath(dragPose3, dragPoseTurn3);
//        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
        specialHPIntake = makeSpinHalfWayPath(dragPoseTurn3, pickup1Pose);
//        specialHPIntake1 = makeLinePath(chamberPose2, pickup12Pose);
        park = makeLinePath(chamberPose5, parkPose);
//        specialHPIntake1score = makeLinePath(pickup12Pose, chamberPose2);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeLinePathThanForwardX(pickup1Pose, chamberPose2,1);
        scorePickup2 = makeLinePathThanForwardX(pickup2Pose, chamberPose3,1);
        scorePickup3 = makeLinePathThanForwardX(pickup3Pose, chamberPose4,1);
        scorePickup4 = makeLinePathThanForwardX(pickup4Pose, chamberPose5,1);
        pickup1 = makeSpinHalfWayPath(chamberPose2, pickup2Pose);
        pickup2 = makeSpinHalfWayPath(chamberPose3, pickup3Pose);
        pickup3 = makeSpinHalfWayPath(chamberPose4, pickup4Pose);
//        pickup4 = makeLinePath(chamberPose5, pickup12Pose);
//        park = makeLinePath(basketScore, parkPose);
//        clawSubsystem.getClawRotationServo().setPosition(0.5);
        new Parallel(
            getResetAngleClaw(),
            getCloseDepositClaw(),
            new Sequential(
                getOpenExtension(),
                getReset(),
                new Wait(0.3),
                getMoveToWall(),
                getV4bWall(),
                getGoToWall(),
                new Wait(0.5),
                getCloseIntakeClaw()
            )
        ).schedule();
        LiftHardware.setPose(0);
        targetPosition = 0;
        follower.setMaxPower(power);
    }

    @Override
    public void myStart() {


        new Sequential(
            new RunNonBlocking(getChamberSeqNoDelay()),
            new Wait(0.04),
            new WaitUntil(()->targetPosition/2<getPose()),
            new InstantCommand(()-> FollowerConstants.drivePIDFCoefficients.P *= 2),
            followPath(scorePreload).raceWith(new Wait(2.1)),
            new InstantCommand(()-> FollowerConstants.drivePIDFCoefficients.P /= 2),
            new Parallel(
                new RunNonBlocking(new Sequential(
                    getOpenDepositClaw(),
                    new WaitUntil(()->follower.getCurrentTValue()>0.6),
                    getOpenPushCommand(),
                    getArmUp(),
                    getTurnLeft(),
                    new RunNonBlocking(getGoToWall())
//                    new RunNonBlocking(getWallSeqSimple())
                )),
                new RunNonBlocking(new RunNonBlocking(
                    new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.95),
                        getExtendoPush()
//                        getAvoidBasket()
                    )
                )),
                followPath(getToDrag1)
            ),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P *= 2),
            followPath(turnDrag1).raceWith(new Wait(0.3)),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P /= 2),
            getArmUp(),
            new RunNonBlocking(new Sequential(
                new WaitUntil(()->follower.getCurrentTValue()>0.99),
                getExtendoPush()
            )),
            followPath(getToDrag2),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P *= 2),
            followPath(turnDrag2).raceWith(new Wait(0.4)),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P /= 2),
            getArmUp(),
            new RunNonBlocking(new Sequential(
                new WaitUntil(()->follower.getCurrentTValue()>0.99),
                getExtendoPush()
            )),
            followPath(getToDrag3),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P *= 2),
            followPath(turnDrag3).raceWith(new Wait(0.4)),
            new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P /= 2),
            new InstantCommand(()-> FollowerConstants.drivePIDFCoefficients.P *= 2),
//            new InstantCommand(()-> FollowerConstants.drivePIDFCoefficients.D = 0.05),
            new InstantCommand(()-> FollowerConstants.translationalPIDFCoefficients.D = 0.04),
            new RunNonBlocking(
                getReset()
            ),
//            new Wait(0.05),
            new Parallel(
//                new Sequential(
//                    new Wait(0.2),
//                    getCloseSlides()
//                ),
                followPath(specialHPIntake).raceWith(
//                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
                    new Wait(0.3),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.1)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup1).raceWith(new Wait(2.1)),
            getUp(),
//            getOpenDepositClaw(),
            new Parallel(
                followPath(pickup1).raceWith(
//                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
//                    new Wait(0.15),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.1)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
//                    new Wait(0.05)
                )
//                new Wait(0.1)
            ),
            getGoToHighChamber(),
            followPath(scorePickup2).raceWith(new Wait(2.1)),
            getUp(),
//            getOpenDepositClaw(),
            new Parallel(
                followPath(pickup2).raceWith(
//                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
//                    new Wait(0.15),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.15)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
//                    new Wait(0.05)
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup3).raceWith(new Wait(2.1)),
            getUp(),
//            getOpenDepositClaw(),
            new Parallel(
                followPath(pickup3).raceWith(
//                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
//                    new Wait(0.15),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.15)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
//                    new Wait(0.05)
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup4).raceWith(new Wait(2.1)),
            getUp(),
//            getOpenDepositClaw(),

            new Parallel(
                new Wait(0.2).then(getOpenCommand()),
                followPath(park)
            ),
            finishAuto
        ).schedule();

    }
    @Override
    public void myStop(){
        LiftVariables.startingPose = getPose();
        DriveVariables.startingAngle = -follower.getPose().getHeading();
//        LiftVariables.startingPose = getPose();
        finishAuto.schedule();
    }
}

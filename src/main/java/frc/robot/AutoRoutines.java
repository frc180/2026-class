package frc.robot;

import com.spamrobotics.util.Helpers;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutines {

    private final AutoFactory factory;
    private final RobotContainer rc;
    private final ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
        rc = RobotContainer.instance;

        // factory.bind("prepare_shoot", prepareShoot());
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = factory.newRoutine("Do Nothing Auto");

        routine.active().onTrue(rc.drivetrain.runOnce(() -> rc.drivetrain.drive(new ChassisSpeeds())));
        return routine;
    }


    public AutoRoutine calibrationDrive2_5m(boolean visionEnabled) {
        final AutoRoutine routine = factory.newRoutine("CalibrationDrive2.5m " + (visionEnabled ? "Vision" : "No Vision"));
        final AutoTrajectory traj = routine.trajectory("DriveStraight_2_5_m_one");

        routine.active().onTrue(Helpers.fastSequence(
            traj.resetOdometry().alongWith(rc.vision.odometryUpdatesEnabled(visionEnabled)),
            followTraj(traj)
        ));

        return routine;
    }

    
    /**
     * Follows a trajectory and then stops the drivetrain at the end.
     */
    private Command followTraj(AutoTrajectory traj) {
        return traj.cmd().finallyDo(() -> rc.drivetrain.drive(zeroSpeeds));
    }

    // private Command finishPath(Pose2d end, BooleanSupplier atEnd) {
    //     return Helpers.fastSequence(
    //         goTrajEnd(end, atEnd).withTimeout(2),
    //         Commands.either(
    //             Commands.none(),
    //             Commands.defer(() -> {
    //                 Pose2d retry = new Pose2d(
    //                     Robot.isBlue() ? 6.5 : Robot.FIELD_LENGTH - 6.5,
    //                     end.getY(),
    //                     end.getRotation()
    //                 );
    //                 return new DriveToPose(rc.drivetrain, retry)
    //                         .withMaxSpeed(0.75)
    //                         .until(atPose(retry))
    //                         .andThen(goTrajEnd(end, atEnd));
    //             }, Set.of(rc.drivetrain)),
    //             atEnd
    //         )
    //     );
    // }

    // private Command goTrajEnd(Pose2d end, BooleanSupplier atEnd) {
    //     return new DriveToPose(rc.drivetrain, end)
    //                         .withMaxSpeed(0.75)
    //                         .until(atEnd);

    // }

    // private BooleanSupplier atPose(Pose2d pose) {
    //     return () -> withinPoseTolerance(
    //         pose,
    //         Meters.of(0.1),
    //         Meters.of(0.1),
    //         Degrees.of(8)
    //     );
    // }

    // private boolean withinPoseTolerance(Pose2d targetPose, Distance xDistance, Distance yDistance, Angle angle) {
    //     return Helpers.withinTolerance(rc.drivetrain.getPose(), targetPose, xDistance, yDistance, angle);
    // }
}
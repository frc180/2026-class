// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.spamrobotics.util.DoubleFunction;
import com.spamrobotics.util.Helpers;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RumbleCommand;

@Logged(importance = Logged.Importance.INFO)
public class RobotContainer {

    public static final double DEADBAND = 0.025;
    public static RobotContainer instance;

    public static final double NO_CAMERA_DEFAULT_DISTANCE = 3.2 - .05;

    private final Telemetry logger = new Telemetry();

    public final CommandXboxController driverController = new CommandXboxController(0);
    private final JoystickInputs inputs = new JoystickInputs();

    // Define all subsystems here
    public final DrivetrainSubsystem drivetrain;

    @Logged(name = "Vision")
    public final VisionSubsystem vision;

    @Logged(name = "Flywheel")
    public final Flywheel flywheel;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    @NotLogged
    private final AutoChooser autoChooser = new AutoChooser();
    @NotLogged
    final SendableChooser<Boolean> feedPriorityChooser = new SendableChooser<>();

    public boolean receivedAutoWinner = false;
    public boolean hasAppliedAutoWinnerOverride = false;
    public boolean autoWinnerBlue = false;
    public boolean choreoWarmedUp = false;

    public Command autoWinnerBlueCommand = Commands.runOnce(() -> {
        autoWinnerBlue = true;
        hasAppliedAutoWinnerOverride = true;
    }).ignoringDisable(true);

    public Command autoWinnerRedCommand = Commands.runOnce(() -> {
        autoWinnerBlue = false;
        hasAppliedAutoWinnerOverride = true;
    }).ignoringDisable(true);


    public RobotContainer() {
        instance = this;
    
        drivetrain = TunerConstants.createDrivetrain();
        vision = new VisionSubsystem();
        flywheel = new Flywheel();

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("My Auto", autoRoutines::doNothing);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putData("Blue Auto Override", autoWinnerBlueCommand);
        SmartDashboard.putData("Red Auto Override", autoWinnerRedCommand);

        configureBindings();
        RobotController.setBrownoutVoltage(6);

        // Choreo warmup
        if (RobotState.isDisabled()) {
            CommandScheduler.getInstance().schedule(Helpers.fastSequence(
                Commands.print("Choreo warmup beginning..."),
                autoFactory.trajectoryCmd("DriveStraight_2_5_m_one").ignoringDisable(true),
                Commands.print("Choreo warmup complete!").alongWith(Commands.runOnce(() -> choreoWarmedUp = true))
            ).ignoringDisable(true));
        }
    }

    private void configureBindings() {
        final Trigger slowMode = driverController.rightBumper();

        final DoubleFunction axisToLinearSpeed = (axis) -> {
            double speed = modifyAxis(axis) * DrivetrainSubsystem.MAX_SPEED;
            if (slowMode.getAsBoolean()) {
                speed *= 0.4;
            }
            return speed;
        };

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(DEADBAND)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1)
                            .transform(axisToLinearSpeed);
        };

        final DoubleSupplier rotationSupplier = () -> {
            double axis = -modifyAxis(driverController.getRightX());
            axis *= 0.8;
            return axis * DrivetrainSubsystem.MAX_ANGULAR_RATE;
        };

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, joystickInputsSupplier, rotationSupplier, slowMode::getAsBoolean).withSlowMode(slowMode));
        driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));
        

        final Trigger activeShiftWarning = new Trigger(() -> {
            double shiftStartTime = Robot.timeUntilNextActiveShift();
            return shiftStartTime >= 0 && shiftStartTime <= 4;
        });

        Command rumbleWarning = Commands.sequence(
            rumble(1).withTimeout(0.25),
            rumble(0).withTimeout(0.75)
        ).repeatedly();

        activeShiftWarning.whileTrue(rumbleWarning);

        final Trigger activeShift = new Trigger(Robot::hubActive);
        activeShift.onTrue(rumble(1).withTimeout(2));


        final Trigger inactiveShiftWarning = new Trigger(() -> {
            double shiftEndTime = Robot.timeUntilNextInactiveShift();
            return shiftEndTime >= 0 && shiftEndTime <= 4;
        });

        Command inactiveRumble = new RumbleCommand(driverController, RumbleType.kRightRumble, 0.1);
        inactiveShiftWarning.onTrue(inactiveRumble.withTimeout(4));

        drivetrain.registerTelemetry(logger::telemeterize);



        Trigger aButton = driverController.a();
        Command spin = flywheel.runSpeed(1);

        aButton.whileTrue(spin);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public Command rumble(double strength) {
        return new RumbleCommand(driverController, RumbleType.kBothRumble, strength);
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }
}

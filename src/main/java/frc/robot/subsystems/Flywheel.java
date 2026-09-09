package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

@Logged
public class Flywheel extends SubsystemBase {
    
    TalonFX motorA;
    TalonFX motorB;

    double targetSpeed = 0;

    // Runs when the Flywheel is created
    public Flywheel() {
        motorA = new TalonFX(15);
        motorB = new TalonFX(16);
    }

    public Command runSpeed(double speed) {
        Runnable action = () -> setSpeed(speed);
        return run(action)
                .finallyDo(() -> setSpeed(0));
    }

    public Command launch() {
        Runnable action = () -> {
            RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
                RobotContainer.instance.drivetrain.getSimPose().getTranslation(),
                Translation2d.kZero, // (0, 0), center of the robot
                new ChassisSpeeds(),
                RobotContainer.instance.drivetrain.getSimPose().getRotation(),
                Inches.of(12),
                FeetPerSecond.of(15),
                Degrees.of(45)
            );
            SimulatedArena.getInstance().addGamePieceProjectile(fuel);
        };
        return runOnce(action);
    }

    // -1 full speed backwards
    // 0 stopped
    // 1 full speed forwards
    private void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
        targetSpeed = speed;
        System.out.println("Motors running at " + speed);
    }
}

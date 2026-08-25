package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    
    TalonFX motorA;
    TalonFX motorB;

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

    // -1 full speed backwards
    // 0 stopped
    // 1 full speed forwards
    private void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
        System.out.println("Motors running at " + speed);
    }
}

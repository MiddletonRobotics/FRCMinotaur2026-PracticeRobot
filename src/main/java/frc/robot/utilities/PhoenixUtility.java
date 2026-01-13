package frc.robot.utilities;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class PhoenixUtility {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for(int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.equals(StatusCode.OK)) break;
        }
    }
}

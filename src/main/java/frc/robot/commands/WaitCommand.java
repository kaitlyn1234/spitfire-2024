import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

public class WaitCommand {
    // Method to create a CompletableFuture with a delay
    public static CompletableFuture<Void> wait(int milliseconds) {
        return CompletableFuture.runAsync(() -> {
            try {
                TimeUnit.MILLISECONDS.sleep(milliseconds);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException(e);
            }
        });
    }

    // Example usage
    public static void main(String[] args) {
        // Create a CompletableFuture representing a wait of 1 second
        CompletableFuture<Void> waitCommand = wait(1000);

        // Chain additional actions after the delay completes
        waitCommand.thenRun(() -> System.out.println("Waited for 1 second"))
                   .thenRun(() -> System.out.println("Now do something else"));
        
        // For testing purposes, wait for all async tasks to complete
        waitCommand.join();
    }
}
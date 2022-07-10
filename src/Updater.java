import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.List;

public class Updater implements ActionListener {
    private List<Task> tasks;
    private final int FPS;
    private int framesCount = 0;
    private int frames = 0;
    private long timePass = System.currentTimeMillis();
    private Timer updater;

    Updater(int FPS) {
        this.FPS = FPS;
        tasks = new LinkedList<>();
        updater = new Timer(1000/FPS, this);
    }

    void addTask(Task task) {
        tasks.add(task);
    }

    void start() {
        updater.start();
    }

    void stop() {
        updater.stop();
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        framesCount++;
        if (timePass + 1000 < System.currentTimeMillis()) {
            frames = framesCount;
            framesCount = 0;
            timePass = System.currentTimeMillis();
        }
        for (Task task : tasks) {
            task.doTask();
        }
    }

    public int getFrames() {
        return frames;
    }

    public int getFPS() {
        return FPS;
    }

}

interface Task {
    void doTask();
}

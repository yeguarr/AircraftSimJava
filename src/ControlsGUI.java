import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class ControlsGUI extends MouseAdapter implements KeyListener {
    private Camera camera;
    private final Set<Integer> pressedKeys = new HashSet<>();
    private Map<Integer, BindKeys> bindKeysMap = new HashMap<>();
    private Matrix.m4x1 mousePosition = new Matrix.m4x1(0,0,0,0);

    public ControlsGUI(Camera camera) {
        this.camera = camera;
    }
    
    public void updateControls() {
        updateMouse();
        updateKeys();
    }

    private void updateMouse() {
        camera.setRotation(camera.getRotation().add( - (mousePosition.getC() - mousePosition.getA())/2. ,(mousePosition.getD() - mousePosition.getB())/2.,0));
        camera.setRotation(new Point3D(Math.min(Math.max(camera.getRotation().getX(),-90),90),camera.getRotation().getY(),camera.getRotation().getZ()));
        mousePosition = new Matrix.m4x1(mousePosition.getC(),mousePosition.getD(),mousePosition.getC(),mousePosition.getD());
    }

    private void updateKeys() {
        if (!pressedKeys.isEmpty()) {
            for (Integer pressedKey : pressedKeys) {
                if (bindKeysMap.containsKey(pressedKey))
                    bindKeysMap.get(pressedKey).update();
            }
        }
    }

    public void mousePressed(MouseEvent e) {
        mousePosition = new Matrix.m4x1(e.getY(),e.getX(),e.getY(),e.getX());
    }

    public void mouseDragged(MouseEvent e) {
        mousePosition = new Matrix.m4x1(mousePosition.getA(),mousePosition.getB(),e.getY(),e.getX());
    }

    public void mouseReleased(MouseEvent e) {
        mousePosition = new Matrix.m4x1(0,0,0,0);
    }

    @Override
    public void keyTyped(KeyEvent e) {
    }

    @Override
    public void keyPressed(KeyEvent e) {
        pressedKeys.add(e.getKeyCode());
    }

    @Override
    public void keyReleased(KeyEvent e) {
        pressedKeys.remove(e.getKeyCode());
    }

    public Camera getCamera() {
        return camera;
    }

    public void bindAKey(int key, BindKeys bindKeyFunction) {
        bindKeysMap.put(key,bindKeyFunction);
    }

    public void bindWASD(Camera camera, double speed) {
        bindKeysMap.put(KeyEvent.VK_W, () -> camera.setPosition(camera.getPosition().add(
                speed*(-Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                speed*Math.sin(Math.toRadians(camera.getRotation().getX())),
                speed*(Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))))));

        bindKeysMap.put(KeyEvent.VK_S, () -> camera.setPosition(camera.getPosition().add(
                speed*(Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                speed*-Math.sin(Math.toRadians(camera.getRotation().getX())),
                speed*(-Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))))));

        bindKeysMap.put(KeyEvent.VK_A, () -> camera.setPosition(camera.getPosition().add(
                speed*Math.cos(Math.toRadians(camera.getRotation().getY())),
                0,
                speed*Math.sin(Math.toRadians(camera.getRotation().getY())))));

        bindKeysMap.put(KeyEvent.VK_D, () -> camera.setPosition(camera.getPosition().add(
                speed* -Math.cos(Math.toRadians(camera.getRotation().getY())),
                0,
                speed* -Math.sin(Math.toRadians(camera.getRotation().getY())))));
    }

}

interface BindKeys {
    void update();
}

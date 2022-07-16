import java.awt.*;
import java.util.LinkedList;

public class LineTracer {
    private LinkedList<Point3D> linesList = new LinkedList<>();
    private Object3D linesObject = new Object3D();
    private int count, delay;

    LineTracer(int count, int delay, Point3D starPosition) {
        for (int i = 0; i < 500; i++) {
            linesList.add(starPosition);
        }
        this.count = count;
        this.delay = delay;
    }

    void updatePosition(Point3D position){
        count++;
        if(count % delay != 0)
            return;
        linesList.pollFirst();
        linesList.add(position);
        Line3D[] line3DS = new Line3D[linesList.size()-2];
        for (int i =0; i < line3DS.length; i++) {
            line3DS[i] = new Line3D(linesList.get(i),linesList.get(i+1),new Color((float) i/line3DS.length,(float)i/line3DS.length,(float)i/line3DS.length), 5);
        }
        linesObject.setFaces(line3DS);
    }

    Object3D getLinesObject() {
        return linesObject;
    }
}

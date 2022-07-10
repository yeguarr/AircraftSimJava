import java.awt.*;
import java.awt.geom.Line2D;

public class Line3D extends Shape3D {
    private Point3D first;
    private Point3D second;
    private Color color;
    private double stroke;

    public Line3D(Point3D first, Point3D second) {
        this.first = first;
        this.second = second;
        this.color = Color.WHITE;
        this.stroke = 1;
    }

    public Line3D(Point3D first, Point3D second, Color color, double stroke) {
        this.first = first;
        this.second = second;
        this.color = color;
        this.stroke = stroke;
    }

    @Override
    public Shape3D getTransformed(Matrix.m4x4 transformation) {
        return new Line3D(transformation.multiply(this.first), transformation.multiply(this.second), color, stroke);
    }

    @Override
    public Polygon getPolygon() {
        return new Polygon(new int[]{(int) this.first.getX(), (int) this.second.getX(), (int)this.second.getX()+1, (int) this.first.getX()+1},
                new int[]{(int) this.first.getY(), (int) this.first.getY(),(int) this.second.getY()+1, (int) this.second.getY()+1}, 4);
    }

    @Override
    public double getZCentre() {
        return (this.first.getZ() + this.second.getZ()) / 2;
    }

    @Override
    public Point3D[] getVertices() {
        return new Point3D[]{first, second};
    }

    @Override
    public Point3D getNormal() {
        return new Point3D(0,0,0);
    }

    @Override
    public Color getColor() {
         return color;
    }

    @Override
    public void drawOnComponent(Graphics2D g2d) {
        g2d.setPaint(this.getColor());
        g2d.setStroke(new BasicStroke((float) Math.abs(stroke)));
        g2d.draw(new Line2D.Double(this.first.getX(),this.first.getY(),this.second.getX(),this.second.getY()));
    }
}

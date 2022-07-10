public class Object3D {
    private Point3D position;
    private Matrix.m4x4 rotation;
    private Point3D centre;
    private Shape3D[] faces;

    public Object3D(Shape3D[] faces) {
        this.faces = faces;
        this.position = new Point3D();
        this.rotation = Utils.eye4x4();
        generateCentre();
    }

    public Object3D() {
        this.faces = new Shape3D[0];
        this.position = new Point3D();
        this.rotation = Utils.eye4x4();
        this.centre = new Point3D();
    }

    public Object3D(Object3D object) {
        this.faces = object.faces.clone();
        this.position = new Point3D(object.position);
        this.rotation = new Matrix.m4x4(object.rotation);
        this.centre = new Point3D(object.centre);
    }

    public Object3D(Shape3D[] faces, Point3D position, Matrix.m4x4 rotation) {
        this.faces = faces;
        this.position = position;
        this.rotation = rotation;
        generateCentre();
    }

    private void generateCentre() {
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        double minZ = Double.MAX_VALUE;
        double maxZ = -Double.MAX_VALUE;

        for (Shape3D shape : faces) {
            for (Point3D vertex : shape.getVertices()){
                if (vertex.getY() > maxY)
                    maxY = vertex.getY();
                if (vertex.getY() < minY)
                    minY = vertex.getY();

                if (vertex.getX() > maxX)
                    maxX = vertex.getX();
                if (vertex.getX() < minX)
                    minX = vertex.getX();

                if (vertex.getZ() > maxZ)
                    maxZ = vertex.getZ();
                if (vertex.getZ() < minZ)
                    minZ = vertex.getZ();
            }
        }
        centre = new Point3D((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);
    }

    public Point3D getPosition() {
        return position;
    }

    public void setPosition(Point3D position) {
        this.position = position;
    }

    public Matrix.m4x4 getRotation() {
        return rotation;
    }

    public void setRotation(Matrix.m4x4 rotation) {
        this.rotation = rotation;
    }

    public Point3D getCentre() {
        return centre;
    }

    public Shape3D[] getFaces() {
        return faces;
    }

    public int getFacesNumber() {
        return faces.length;
    }

    public void setFaces(Shape3D[] faces) {
        this.faces = faces;
    }
}

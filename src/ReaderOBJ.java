import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;
import java.util.List;

public class ReaderOBJ {
    private List<Point3D> vertices = new LinkedList<>();
    private List<Shape3D> faces = new LinkedList<>();
    private Map<String, Color> materials = new HashMap<>();

    public ReaderOBJ(String file) {
        File text = new File(file);
        try {
            Scanner reader = new Scanner(text);
            Color currentMaterial = Color.WHITE;
            while(reader.hasNextLine()) {
                String[] data = reader.nextLine().trim().replaceAll(" +", " ").split(" ");
                switch (data[0]) {
                    case "v":
                        if (data.length == 5) {
                            double x = Double.parseDouble(data[1]);
                            double y = -Double.parseDouble(data[2]);
                            double z = Double.parseDouble(data[3]);
                            double w = Double.parseDouble(data[4]);
                            vertices.add(new Point3D(x, y, z, w));
                        } else if (data.length == 4) {
                            double x = Double.parseDouble(data[1]);
                            double y = -Double.parseDouble(data[2]);
                            double z = Double.parseDouble(data[3]);
                            vertices.add(new Point3D(x, y, z));
                        }
                        break;
                    case "f":
                        if (data.length == 4) {
                            int first = getCorrectV(Integer.parseInt(data[1].split("/")[0])) - 1;
                            int second = getCorrectV(Integer.parseInt(data[2].split("/")[0])) - 1;
                            int third = getCorrectV(Integer.parseInt(data[3].split("/")[0])) - 1;
                            faces.add(new Triangle3D(vertices.get(first), vertices.get(second), vertices.get(third), currentMaterial));
                        } else {
                            List<Point3D> buff = new LinkedList<>();
                            for (int i = 1; i < data.length; i++) {
                                int pos = getCorrectV(Integer.parseInt(data[i].split("/")[0])) - 1;
                                buff.add(vertices.get(pos));
                            }
                            Point3D first = buff.get(0);
                            for (int i = 1; i < buff.size() - 1; i++) {
                                faces.add(new Triangle3D(first, buff.get(i), buff.get(i + 1), currentMaterial));
                            }
                        }
                        break;
                    case "mtllib":
                        readMTL(data[1]);
                        break;
                    case "usemtl":
                        currentMaterial = materials.get(data[1]);
                        break;
                }
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public Object3D getObject() {
        return new Object3D(faces.toArray(new Shape3D[0]));
    }

    private void readMTL(String file) {
        File text = new File(file);
        try {
            Scanner reader = new Scanner(text);
            String materialName = "";
            float r = 1;
            float g = 1;
            float b = 1;
            float a = 1;
            while(reader.hasNextLine()) {
                String[] data = reader.nextLine().trim().replaceAll(" +", " ").split(" ");
                switch (data[0]) {
                    case "newmtl":
                        materials.put(materialName, new Color(r, g, b, a));
                        r = 1;
                        g = 1;
                        b = 1;
                        a = 1;
                        materialName = data[1];
                        break;
                    case "Kd":
                        r = Float.parseFloat(data[1]);
                        g = Float.parseFloat(data[2]);
                        b = Float.parseFloat(data[3]);
                        break;
                    case "d":
                    case "Tr":
                        a = Float.parseFloat(data[1]);
                        break;
                }
            }
            materials.put(materialName, new Color(r, g, b, a));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }

    private int getCorrectV(int i) {
        if (i > 0)
            return i;
        else
            return vertices.size() + i + 1;
    }
}

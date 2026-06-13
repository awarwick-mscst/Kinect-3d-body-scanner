using System;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Text;
using Microsoft.Kinect.Fusion;

namespace KinectScanner
{
    /// <summary>
    /// Writes a Kinect Fusion ColorMesh to disk. Vertices are in meters.
    /// Y and Z are flipped on export so the model is upright (Y-up, right-handed)
    /// in standard mesh viewers; flipping two axes preserves triangle winding.
    /// </summary>
    public static class MeshExporter
    {
        public static void Save(ColorMesh mesh, string path, bool withColor, out int vertexCount, out int triangleCount)
        {
            Vector3[] vertices = ToArray(mesh.GetVertices());
            Vector3[] normals = ToArray(mesh.GetNormals());
            int[] indices = ToArray(mesh.GetTriangleIndexes());
            int[] colors = withColor ? ToArray(mesh.GetColors()) : null;

            vertexCount = vertices.Length;
            triangleCount = indices.Length / 3;

            string ext = Path.GetExtension(path).ToLowerInvariant();
            switch (ext)
            {
                case ".ply":
                    SavePly(path, vertices, normals, indices, colors);
                    break;
                case ".obj":
                    SaveObj(path, vertices, normals, indices, colors);
                    break;
                case ".stl":
                    SaveStl(path, vertices, indices);
                    break;
                default:
                    throw new ArgumentException("Unsupported mesh format: " + ext);
            }
        }

        // ---------- PLY (binary little-endian, optional vertex color) ----------

        private static void SavePly(string path, Vector3[] vertices, Vector3[] normals, int[] indices, int[] colors)
        {
            using (var stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
            using (var writer = new BinaryWriter(stream))
            {
                var header = new StringBuilder();
                header.Append("ply\n");
                header.Append("format binary_little_endian 1.0\n");
                header.Append("comment Created by Kinect 3D Scanner\n");
                header.Append("element vertex ").Append(vertices.Length).Append('\n');
                header.Append("property float x\nproperty float y\nproperty float z\n");
                header.Append("property float nx\nproperty float ny\nproperty float nz\n");
                if (colors != null)
                {
                    header.Append("property uchar red\nproperty uchar green\nproperty uchar blue\n");
                }
                header.Append("element face ").Append(indices.Length / 3).Append('\n');
                header.Append("property list uchar int vertex_indices\n");
                header.Append("end_header\n");
                writer.Write(Encoding.ASCII.GetBytes(header.ToString()));

                for (int i = 0; i < vertices.Length; i++)
                {
                    writer.Write(vertices[i].X);
                    writer.Write(-vertices[i].Y);
                    writer.Write(-vertices[i].Z);
                    writer.Write(normals[i].X);
                    writer.Write(-normals[i].Y);
                    writer.Write(-normals[i].Z);
                    if (colors != null)
                    {
                        int c = colors[i];
                        writer.Write((byte)((c >> 16) & 0xFF)); // red
                        writer.Write((byte)((c >> 8) & 0xFF));  // green
                        writer.Write((byte)(c & 0xFF));         // blue
                    }
                }

                for (int i = 0; i + 2 < indices.Length; i += 3)
                {
                    writer.Write((byte)3);
                    writer.Write(indices[i]);
                    writer.Write(indices[i + 1]);
                    writer.Write(indices[i + 2]);
                }
            }
        }

        // ---------- OBJ (ASCII; vertex colors as non-standard "v x y z r g b") ----------

        private static void SaveObj(string path, Vector3[] vertices, Vector3[] normals, int[] indices, int[] colors)
        {
            var ci = CultureInfo.InvariantCulture;
            using (var writer = new StreamWriter(path, false, Encoding.ASCII, 1 << 20))
            {
                writer.WriteLine("# Created by Kinect 3D Scanner (units: meters)");

                var sb = new StringBuilder(96);
                for (int i = 0; i < vertices.Length; i++)
                {
                    sb.Length = 0;
                    sb.Append("v ");
                    sb.Append(vertices[i].X.ToString("R", ci)).Append(' ');
                    sb.Append((-vertices[i].Y).ToString("R", ci)).Append(' ');
                    sb.Append((-vertices[i].Z).ToString("R", ci));
                    if (colors != null)
                    {
                        int c = colors[i];
                        sb.Append(' ').Append((((c >> 16) & 0xFF) / 255f).ToString("0.###", ci));
                        sb.Append(' ').Append((((c >> 8) & 0xFF) / 255f).ToString("0.###", ci));
                        sb.Append(' ').Append(((c & 0xFF) / 255f).ToString("0.###", ci));
                    }
                    writer.WriteLine(sb.ToString());
                }

                for (int i = 0; i < normals.Length; i++)
                {
                    sb.Length = 0;
                    sb.Append("vn ");
                    sb.Append(normals[i].X.ToString("R", ci)).Append(' ');
                    sb.Append((-normals[i].Y).ToString("R", ci)).Append(' ');
                    sb.Append((-normals[i].Z).ToString("R", ci));
                    writer.WriteLine(sb.ToString());
                }

                for (int i = 0; i + 2 < indices.Length; i += 3)
                {
                    int a = indices[i] + 1, b = indices[i + 1] + 1, c2 = indices[i + 2] + 1;
                    writer.WriteLine("f " + a + "//" + a + " " + b + "//" + b + " " + c2 + "//" + c2);
                }
            }
        }

        // ---------- STL (binary; no color — units stay in meters) ----------

        private static void SaveStl(string path, Vector3[] vertices, int[] indices)
        {
            using (var stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
            using (var writer = new BinaryWriter(stream))
            {
                var header = new byte[80];
                byte[] tag = Encoding.ASCII.GetBytes("Kinect 3D Scanner binary STL (meters)");
                Array.Copy(tag, header, Math.Min(tag.Length, 80));
                writer.Write(header);
                writer.Write((uint)(indices.Length / 3));

                for (int i = 0; i + 2 < indices.Length; i += 3)
                {
                    Vector3 a = Flip(vertices[indices[i]]);
                    Vector3 b = Flip(vertices[indices[i + 1]]);
                    Vector3 c = Flip(vertices[indices[i + 2]]);

                    float ux = b.X - a.X, uy = b.Y - a.Y, uz = b.Z - a.Z;
                    float vx = c.X - a.X, vy = c.Y - a.Y, vz = c.Z - a.Z;
                    float nx = uy * vz - uz * vy;
                    float ny = uz * vx - ux * vz;
                    float nz = ux * vy - uy * vx;
                    float len = (float)Math.Sqrt(nx * nx + ny * ny + nz * nz);
                    if (len > 1e-12f) { nx /= len; ny /= len; nz /= len; }
                    else { nx = 0f; ny = 0f; nz = 0f; }

                    writer.Write(nx); writer.Write(ny); writer.Write(nz);
                    writer.Write(a.X); writer.Write(a.Y); writer.Write(a.Z);
                    writer.Write(b.X); writer.Write(b.Y); writer.Write(b.Z);
                    writer.Write(c.X); writer.Write(c.Y); writer.Write(c.Z);
                    writer.Write((ushort)0);
                }
            }
        }

        private static Vector3 Flip(Vector3 v)
        {
            v.Y = -v.Y;
            v.Z = -v.Z;
            return v;
        }

        private static T[] ToArray<T>(ReadOnlyCollection<T> collection)
        {
            var array = new T[collection.Count];
            collection.CopyTo(array, 0);
            return array;
        }
    }
}

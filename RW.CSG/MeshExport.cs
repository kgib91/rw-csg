using System.Numerics;

namespace RW.CSG;

/// <summary>
/// Iso-surface extraction from the sparse SDF volume.
/// Uses marching tetrahedra (inside each voxel cube) to avoid large lookup tables.
/// </summary>
public static class MeshExport
{
    public readonly record struct Triangle(int A, int B, int C);

    public sealed class Mesh
    {
        public readonly List<Vector3> Vertices = [];
        public readonly List<Vector3> Normals = [];
        public readonly List<Triangle> Triangles = [];
    }

    private static readonly int[,] CubeCorners =
    {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
        { 1, 0, 1 },
        { 1, 1, 1 },
        { 0, 1, 1 }
    };

    // 6 tetrahedra that partition the cube.
    private static readonly int[,] Tets =
    {
        { 0, 5, 1, 6 },
        { 0, 5, 6, 4 },
        { 0, 2, 6, 1 },
        { 0, 2, 3, 6 },
        { 0, 7, 6, 3 },
        { 0, 4, 6, 7 }
    };

    private static readonly int[,] TetEdges =
    {
        { 0, 1 },
        { 0, 2 },
        { 0, 3 },
        { 1, 2 },
        { 1, 3 },
        { 2, 3 }
    };

    /// <summary>
    /// Extracts an iso-surface at d=0 over the full volume.
    /// Step > 1 performs coarse decimation by skipping voxels.
    /// </summary>
    public static Mesh Extract(SparseSdfVolume volume, int step = 1)
    {
        if (step <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(step));
        }

        var mesh = new Mesh();
        float vox = volume.VoxelSize;
        float h = 0.5f * vox;

        var p = new Vector3[8];
        var d = new float[8];

        for (int z = 0; z < volume.VoxelsZ - step; z += step)
        {
            for (int y = 0; y < volume.VoxelsY - step; y += step)
            {
                for (int x = 0; x < volume.VoxelsX - step; x += step)
                {
                    bool hasNeg = false;
                    bool hasPos = false;

                    for (int c = 0; c < 8; c++)
                    {
                        int cx = x + CubeCorners[c, 0] * step;
                        int cy = y + CubeCorners[c, 1] * step;
                        int cz = z + CubeCorners[c, 2] * step;
                        float wx = (cx + 0.5f) * vox;
                        float wy = (cy + 0.5f) * vox;
                        float wz = (cz + 0.5f) * vox;
                        p[c] = new Vector3(wx, wy, wz);
                        d[c] = volume.GetVoxel(cx, cy, cz);
                        if (d[c] < 0) hasNeg = true; else hasPos = true;
                    }

                    if (!(hasNeg && hasPos))
                    {
                        continue;
                    }

                    for (int t = 0; t < 6; t++)
                    {
                        PolygonizeTet(volume, p, d, t, h, mesh);
                    }
                }
            }
        }

        return mesh;
    }

    private static void PolygonizeTet(SparseSdfVolume volume, Vector3[] p, float[] d, int tetIndex, float h, Mesh mesh)
    {
        int i0 = Tets[tetIndex, 0];
        int i1 = Tets[tetIndex, 1];
        int i2 = Tets[tetIndex, 2];
        int i3 = Tets[tetIndex, 3];

        int[] ids = [i0, i1, i2, i3];
        Span<Vector3> ip = stackalloc Vector3[6];
        Span<Vector3> inorm = stackalloc Vector3[6];
        int nEdgeHits = 0;

        for (int e = 0; e < 6; e++)
        {
            int a = ids[TetEdges[e, 0]];
            int b = ids[TetEdges[e, 1]];
            float da = d[a];
            float db = d[b];
            if ((da < 0) == (db < 0))
            {
                continue;
            }

            float t = da / (da - db);
            Vector3 q = Vector3.Lerp(p[a], p[b], t);
            ip[nEdgeHits] = q;
            inorm[nEdgeHits] = EstimateNormal(volume, q, h);
            nEdgeHits++;
        }

        if (nEdgeHits < 3)
        {
            return;
        }

        if (nEdgeHits == 3)
        {
            AddTri(mesh, ip[0], ip[1], ip[2], inorm[0], inorm[1], inorm[2]);
            return;
        }

        // nEdgeHits == 4 : split quad into two triangles.
        AddTri(mesh, ip[0], ip[1], ip[2], inorm[0], inorm[1], inorm[2]);
        AddTri(mesh, ip[0], ip[2], ip[3], inorm[0], inorm[2], inorm[3]);
    }

    private static Vector3 EstimateNormal(SparseSdfVolume volume, Vector3 p, float h)
    {
        float gx = volume.SampleFine(p.X + h, p.Y, p.Z) - volume.SampleFine(p.X - h, p.Y, p.Z);
        float gy = volume.SampleFine(p.X, p.Y + h, p.Z) - volume.SampleFine(p.X, p.Y - h, p.Z);
        float gz = volume.SampleFine(p.X, p.Y, p.Z + h) - volume.SampleFine(p.X, p.Y, p.Z - h);
        var n = new Vector3(gx, gy, gz);
        float len = n.Length();
        return len > 1e-9f ? n / len : Vector3.UnitZ;
    }

    private static void AddTri(Mesh mesh, Vector3 a, Vector3 b, Vector3 c, Vector3 na, Vector3 nb, Vector3 nc)
    {
        int baseIndex = mesh.Vertices.Count;
        mesh.Vertices.Add(a);
        mesh.Vertices.Add(b);
        mesh.Vertices.Add(c);
        mesh.Normals.Add(na);
        mesh.Normals.Add(nb);
        mesh.Normals.Add(nc);
        mesh.Triangles.Add(new Triangle(baseIndex, baseIndex + 1, baseIndex + 2));
    }
}

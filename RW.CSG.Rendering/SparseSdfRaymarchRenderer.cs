using System.Numerics;
using Silk.NET.OpenGL;

namespace RW.CSG.Rendering;

public enum SparseSdfRaymarchDebugView
{
    Shaded,
    HitClassification,
    MarchCost,
    BrickTopology,
    VoxelPhase,
    SdfResidual,
    SurfaceNormals
}

public static class SparseSdfRaymarchDebugViews
{
    private static readonly string[] ViewNames =
    [
        "Shaded",
        "Hit classification",
        "March cost",
        "Brick topology",
        "Voxel phase",
        "SDF residual",
        "Surface normals"
    ];

    private static readonly string[] ViewLegends =
    [
        "Shaded material preview",
        "Blue solid | Green filtered | White raw | Red near miss",
        "Blue low cost to cyan to green to yellow to red high cost",
        "Stable pseudo-color per 8x8x8 brick; white marks brick planes",
        "RGB is local XYZ phase inside each voxel; white marks voxel planes",
        "Raw field at reconstructed hit: blue inside | white zero | red outside",
        "RGB encodes the XYZ normal from [-1, +1]"
    ];

    public static IReadOnlyList<string> Names => ViewNames;

    public static string GetName(SparseSdfRaymarchDebugView view)
        => ViewNames[Math.Clamp((int)view, 0, ViewNames.Length - 1)];

    public static string GetLegend(SparseSdfRaymarchDebugView view)
        => ViewLegends[Math.Clamp((int)view, 0, ViewLegends.Length - 1)];
}

/// <summary>
/// Draw parameters for <see cref="SparseSdfRaymarchRenderer"/>. The sparse volume remains in
/// RW.CSG coordinates; the two transform matrices place it in the host application's world.
/// </summary>
public readonly record struct SparseSdfRaymarchRenderRequest(
    Matrix4x4 InverseViewProjection,
    Matrix4x4 ViewProjection,
    Matrix4x4 VolumeToWorld,
    Matrix4x4 WorldToVolume,
    Vector3 VisibleStockSize,
    Vector4 MaterialColor,
    Vector3 LocalLightDirection,
    int Width,
    int Height,
    SparseSdfRaymarchDebugView DebugView = SparseSdfRaymarchDebugView.Shaded);

/// <summary>Details of the most recent full sparse-volume texture upload.</summary>
public readonly record struct SparseSdfRaymarchUploadInfo(
    int MixedBrickCount,
    int TilesPerAxis,
    int AtlasTexelsPerAxis,
    Vector3 VolumeSize,
    float VoxelSize);

/// <summary>Details of an incremental sparse-volume texture update.</summary>
public readonly record struct SparseSdfRaymarchUpdateInfo(
    int DirtyBrickCount,
    int MixedBrickUploads,
    int NewAtlasTiles,
    bool AtlasResized,
    int AtlasTexelsPerAxis);

/// <summary>
/// Reusable OpenGL renderer for <see cref="SparseSdfVolume"/>. It uploads a sparse brick atlas,
/// brick indirection texture, and coarse SDF, then raymarches the field into the caller's current
/// framebuffer. It does not own a window, camera, framebuffer, or UI framework.
/// </summary>
public sealed class SparseSdfRaymarchRenderer : IDisposable
{
    private const int R16f = 0x822D;
    private const int HalfFloat = 0x140B;

    private readonly GL _gl;
    private uint _program;
    private uint _vao;
    private uint _vbo;
    private uint _atlasTexture;
    private uint _indirectionTexture;
    private uint _coarseTexture;

    private int[] _brickToTile = [];
    private byte[] _indirectionPixels = [];
    private readonly Stack<int> _freeAtlasTiles = new();
    private int _nextAtlasTile;
    private int _tilesPerAxis;
    private int _bricksX;
    private int _bricksY;
    private int _bricksZ;

    private int _inverseViewProjectionLocation;
    private int _viewProjectionLocation;
    private int _volumeToWorldLocation;
    private int _worldToVolumeLocation;
    private int _materialColorLocation;
    private int _lightDirectionLocation;
    private int _atlasLocation;
    private int _indirectionLocation;
    private int _coarseLocation;
    private int _stockLocation;
    private int _volumeLocation;
    private int _voxelLocation;
    private int _bandLocation;
    private int _brickLocation;
    private int _atlasTexelsLocation;
    private int _resolutionLocation;
    private int _debugViewLocation;

    private Vector3 _volumeSize;
    private float _voxelSize;
    private float _bandSize;
    private int _atlasTexelsPerAxis;
    private bool _hasVolume;
    private bool _disposed;

    public SparseSdfRaymarchRenderer(GL gl)
    {
        _gl = gl ?? throw new ArgumentNullException(nameof(gl));
        InitializeProgram();
        InitializeFullscreenQuad();
    }

    public bool HasVolume => _hasVolume;

    public SparseSdfRaymarchUploadInfo Upload(SparseSdfVolume volume)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentNullException.ThrowIfNull(volume);

        _volumeSize = new Vector3(volume.VolumeSizeX, volume.VolumeSizeY, volume.VolumeSizeZ);
        _voxelSize = volume.VoxelSize;
        _bandSize = volume.BandWorld;

        var mixedBricks = new List<int>();
        for (int brick = 0; brick < volume.BrickCount; brick++)
        {
            if (volume.BrickState(brick) >= 0)
                mixedBricks.Add(brick);
        }

        int requiredTilesPerAxis = Math.Max(1, (int)Math.Ceiling(Math.Cbrt(Math.Max(mixedBricks.Count, 1))));
        bool sameLayout = volume.BricksX == _bricksX
            && volume.BricksY == _bricksY
            && volume.BricksZ == _bricksZ;
        int tilesPerAxis = Math.Max(requiredTilesPerAxis, sameLayout ? _tilesPerAxis : 0);
        tilesPerAxis = Math.Max(4, (tilesPerAxis + 3) / 4 * 4);
        if (tilesPerAxis > byte.MaxValue)
            throw new InvalidOperationException("The sparse SDF atlas exceeds the 8-bit tile indirection range.");

        _tilesPerAxis = tilesPerAxis;
        _atlasTexelsPerAxis = checked(tilesPerAxis * 9);
        var atlas = new ushort[checked(_atlasTexelsPerAxis * _atlasTexelsPerAxis * _atlasTexelsPerAxis)];
        _indirectionPixels = new byte[checked(volume.BrickCount * 4)];
        _brickToTile = new int[volume.BrickCount];
        Array.Fill(_brickToTile, -1);
        _freeAtlasTiles.Clear();
        _nextAtlasTile = mixedBricks.Count;
        _bricksX = volume.BricksX;
        _bricksY = volume.BricksY;
        _bricksZ = volume.BricksZ;

        for (int mixedIndex = 0; mixedIndex < mixedBricks.Count; mixedIndex++)
        {
            int brick = mixedBricks[mixedIndex];
            int brickX = brick % volume.BricksX;
            int brickY = brick / volume.BricksX % volume.BricksY;
            int brickZ = brick / (volume.BricksX * volume.BricksY);
            int tileX = mixedIndex % tilesPerAxis;
            int tileY = mixedIndex / tilesPerAxis % tilesPerAxis;
            int tileZ = mixedIndex / (tilesPerAxis * tilesPerAxis);

            _brickToTile[brick] = mixedIndex;
            int indirectionOffset = brick * 4;
            _indirectionPixels[indirectionOffset] = (byte)tileX;
            _indirectionPixels[indirectionOffset + 1] = (byte)tileY;
            _indirectionPixels[indirectionOffset + 2] = (byte)tileZ;
            _indirectionPixels[indirectionOffset + 3] = byte.MaxValue;

            for (int z = 0; z < 9; z++)
            {
                for (int y = 0; y < 9; y++)
                {
                    for (int x = 0; x < 9; x++)
                    {
                        int atlasIndex = ((tileZ * 9 + z) * _atlasTexelsPerAxis + tileY * 9 + y)
                            * _atlasTexelsPerAxis + tileX * 9 + x;
                        atlas[atlasIndex] = BitConverter.HalfToUInt16Bits(
                            (Half)volume.GetVoxel(brickX * 8 + x, brickY * 8 + y, brickZ * 8 + z));
                    }
                }
            }
        }

        for (int brick = 0; brick < volume.BrickCount; brick++)
        {
            if (volume.BrickState(brick) == SparseSdfVolume.Solid)
                _indirectionPixels[brick * 4 + 3] = 128;
        }

        var coarse = new ushort[volume.Coarse.Length];
        for (int i = 0; i < coarse.Length; i++)
            coarse[i] = BitConverter.HalfToUInt16Bits((Half)volume.Coarse[i]);

        _gl.PixelStore(PixelStoreParameter.UnpackAlignment, 1);
        UploadR16fTexture(
            ref _atlasTexture,
            _atlasTexelsPerAxis,
            _atlasTexelsPerAxis,
            _atlasTexelsPerAxis,
            atlas,
            linear: true);
        UploadRgba8Texture(
            ref _indirectionTexture,
            volume.BricksX,
            volume.BricksY,
            volume.BricksZ,
            _indirectionPixels);
        UploadR16fTexture(
            ref _coarseTexture,
            volume.CoarseX,
            volume.CoarseY,
            volume.CoarseZ,
            coarse,
            linear: true);

        _hasVolume = true;
        return new SparseSdfRaymarchUploadInfo(
            mixedBricks.Count,
            tilesPerAxis,
            _atlasTexelsPerAxis,
            _volumeSize,
            _voxelSize);
    }

    public unsafe SparseSdfRaymarchUpdateInfo UploadChanges(
        SparseSdfVolume volume,
        ReadOnlySpan<int> dirtyBricks)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentNullException.ThrowIfNull(volume);

        if (!_hasVolume
            || volume.BricksX != _bricksX
            || volume.BricksY != _bricksY
            || volume.BricksZ != _bricksZ
            || volume.VoxelSize != _voxelSize)
        {
            SparseSdfRaymarchUploadInfo upload = Upload(volume);
            return new SparseSdfRaymarchUpdateInfo(
                dirtyBricks.Length,
                upload.MixedBrickCount,
                upload.MixedBrickCount,
                true,
                upload.AtlasTexelsPerAxis);
        }

        if (dirtyBricks.IsEmpty)
        {
            return new SparseSdfRaymarchUpdateInfo(0, 0, 0, false, _atlasTexelsPerAxis);
        }

        for (int i = 0; i < dirtyBricks.Length; i++)
        {
            int brick = dirtyBricks[i];
            if ((uint)brick >= (uint)_brickToTile.Length)
                continue;

            if (volume.BrickState(brick) < 0 && _brickToTile[brick] >= 0)
                ReleaseAtlasTile(brick);
        }

        bool atlasResized = false;
        int mixedUploads = 0;
        int newTiles = 0;
        _gl.BindTexture(TextureTarget.Texture3D, _atlasTexture);
        for (int i = 0; i < dirtyBricks.Length; i++)
        {
            int brick = dirtyBricks[i];
            if ((uint)brick >= (uint)_brickToTile.Length)
                continue;

            int state = volume.BrickState(brick);
            int tile = _brickToTile[brick];
            if (state >= 0)
            {
                if (tile < 0)
                {
                    tile = AllocateAtlasTile(volume, ref atlasResized);
                    _brickToTile[brick] = tile;
                    newTiles++;
                }

                _gl.BindTexture(TextureTarget.Texture3D, _atlasTexture);
                UploadAtlasTile(volume, brick, tile);
                mixedUploads++;
            }

            UpdateIndirection(brick, state, tile);
        }
        _gl.BindTexture(TextureTarget.Texture3D, 0);

        UpdateRgba8Texture(
            _indirectionTexture,
            volume.BricksX,
            volume.BricksY,
            volume.BricksZ,
            _indirectionPixels);
        UpdateCoarseTexture(volume);

        return new SparseSdfRaymarchUpdateInfo(
            dirtyBricks.Length,
            mixedUploads,
            newTiles,
            atlasResized,
            _atlasTexelsPerAxis);
    }

    public unsafe void Render(in SparseSdfRaymarchRenderRequest request)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_hasVolume || request.Width <= 0 || request.Height <= 0)
            return;

        Matrix4x4 inverseViewProjection = request.InverseViewProjection;
        Matrix4x4 viewProjection = request.ViewProjection;
        Matrix4x4 volumeToWorld = request.VolumeToWorld;
        Matrix4x4 worldToVolume = request.WorldToVolume;

        _gl.UseProgram(_program);
        _gl.UniformMatrix4(_inverseViewProjectionLocation, 1, false, (float*)&inverseViewProjection);
        _gl.UniformMatrix4(_viewProjectionLocation, 1, false, (float*)&viewProjection);
        _gl.UniformMatrix4(_volumeToWorldLocation, 1, false, (float*)&volumeToWorld);
        _gl.UniformMatrix4(_worldToVolumeLocation, 1, false, (float*)&worldToVolume);
        _gl.Uniform4(
            _materialColorLocation,
            request.MaterialColor.X,
            request.MaterialColor.Y,
            request.MaterialColor.Z,
            request.MaterialColor.W);
        _gl.Uniform3(
            _lightDirectionLocation,
            request.LocalLightDirection.X,
            request.LocalLightDirection.Y,
            request.LocalLightDirection.Z);
        _gl.Uniform3(_stockLocation, request.VisibleStockSize.X, request.VisibleStockSize.Y, request.VisibleStockSize.Z);
        _gl.Uniform3(_volumeLocation, _volumeSize.X, _volumeSize.Y, _volumeSize.Z);
        _gl.Uniform1(_voxelLocation, _voxelSize);
        _gl.Uniform1(_bandLocation, _bandSize);
        _gl.Uniform1(_brickLocation, 8f * _voxelSize);
        _gl.Uniform1(_atlasTexelsLocation, (float)_atlasTexelsPerAxis);
        _gl.Uniform2(_resolutionLocation, (float)request.Width, (float)request.Height);
        _gl.Uniform1(_debugViewLocation, (int)request.DebugView);

        _gl.ActiveTexture(TextureUnit.Texture0);
        _gl.BindTexture(TextureTarget.Texture3D, _atlasTexture);
        _gl.Uniform1(_atlasLocation, 0);

        _gl.ActiveTexture(TextureUnit.Texture1);
        _gl.BindTexture(TextureTarget.Texture3D, _indirectionTexture);
        _gl.Uniform1(_indirectionLocation, 1);

        _gl.ActiveTexture(TextureUnit.Texture2);
        _gl.BindTexture(TextureTarget.Texture3D, _coarseTexture);
        _gl.Uniform1(_coarseLocation, 2);

        _gl.BindVertexArray(_vao);
        _gl.DrawArrays(PrimitiveType.Triangles, 0, 6);
        _gl.BindVertexArray(0);

        _gl.ActiveTexture(TextureUnit.Texture2);
        _gl.BindTexture(TextureTarget.Texture3D, 0);
        _gl.ActiveTexture(TextureUnit.Texture1);
        _gl.BindTexture(TextureTarget.Texture3D, 0);
        _gl.ActiveTexture(TextureUnit.Texture0);
        _gl.BindTexture(TextureTarget.Texture3D, 0);
        _gl.UseProgram(0);
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        _gl.DeleteProgram(_program);
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteVertexArray(_vao);
        _gl.DeleteTexture(_atlasTexture);
        _gl.DeleteTexture(_indirectionTexture);
        _gl.DeleteTexture(_coarseTexture);

        _program = 0;
        _vbo = 0;
        _vao = 0;
        _atlasTexture = 0;
        _indirectionTexture = 0;
        _coarseTexture = 0;
        _brickToTile = [];
        _indirectionPixels = [];
        _freeAtlasTiles.Clear();
        _nextAtlasTile = 0;
        _tilesPerAxis = 0;
        _bricksX = 0;
        _bricksY = 0;
        _bricksZ = 0;
        _hasVolume = false;
        _disposed = true;
    }

    private int AllocateAtlasTile(SparseSdfVolume volume, ref bool atlasResized)
    {
        if (_freeAtlasTiles.TryPop(out int recycled))
            return recycled;

        int capacity = checked(_tilesPerAxis * _tilesPerAxis * _tilesPerAxis);
        if (_nextAtlasTile >= capacity)
        {
            ReclaimUniformAtlasTiles(volume);
            if (_freeAtlasTiles.TryPop(out recycled))
                return recycled;

            GrowAtlas(volume);
            atlasResized = true;
        }

        return _nextAtlasTile++;
    }

    private void ReleaseAtlasTile(int brick)
    {
        int tile = _brickToTile[brick];
        if (tile < 0)
            return;

        _brickToTile[brick] = -1;
        _freeAtlasTiles.Push(tile);
    }

    private void ReclaimUniformAtlasTiles(SparseSdfVolume volume)
    {
        for (int brick = 0; brick < _brickToTile.Length; brick++)
        {
            if (_brickToTile[brick] >= 0 && volume.BrickState(brick) < 0)
                ReleaseAtlasTile(brick);
        }
    }

    private unsafe void GrowAtlas(SparseSdfVolume volume)
    {
        int newTilesPerAxis = checked(_tilesPerAxis + 4);
        if (newTilesPerAxis > byte.MaxValue)
            throw new InvalidOperationException("The sparse SDF atlas exceeds the 8-bit tile indirection range.");

        int newAtlasTexels = checked(newTilesPerAxis * 9);
        var atlas = new ushort[checked(newAtlasTexels * newAtlasTexels * newAtlasTexels)];
        for (int brick = 0; brick < _brickToTile.Length; brick++)
        {
            int tile = _brickToTile[brick];
            if (tile < 0)
                continue;

            int brickX = brick % volume.BricksX;
            int brickY = brick / volume.BricksX % volume.BricksY;
            int brickZ = brick / (volume.BricksX * volume.BricksY);
            int tileX = tile % newTilesPerAxis;
            int tileY = tile / newTilesPerAxis % newTilesPerAxis;
            int tileZ = tile / (newTilesPerAxis * newTilesPerAxis);
            for (int z = 0; z < 9; z++)
            {
                for (int y = 0; y < 9; y++)
                {
                    for (int x = 0; x < 9; x++)
                    {
                        int atlasIndex = ((tileZ * 9 + z) * newAtlasTexels + tileY * 9 + y)
                            * newAtlasTexels + tileX * 9 + x;
                        atlas[atlasIndex] = BitConverter.HalfToUInt16Bits(
                            (Half)volume.GetVoxel(brickX * 8 + x, brickY * 8 + y, brickZ * 8 + z));
                    }
                }
            }
        }

        _tilesPerAxis = newTilesPerAxis;
        _atlasTexelsPerAxis = newAtlasTexels;
        for (int brick = 0; brick < _brickToTile.Length; brick++)
            UpdateIndirection(brick, volume.BrickState(brick), _brickToTile[brick]);

        UploadR16fTexture(
            ref _atlasTexture,
            _atlasTexelsPerAxis,
            _atlasTexelsPerAxis,
            _atlasTexelsPerAxis,
            atlas,
            linear: true);
        UploadRgba8Texture(
            ref _indirectionTexture,
            volume.BricksX,
            volume.BricksY,
            volume.BricksZ,
            _indirectionPixels);
    }

    private void UpdateIndirection(int brick, int state, int tile)
    {
        int offset = brick * 4;
        if (state >= 0 && tile >= 0)
        {
            _indirectionPixels[offset] = (byte)(tile % _tilesPerAxis);
            _indirectionPixels[offset + 1] = (byte)(tile / _tilesPerAxis % _tilesPerAxis);
            _indirectionPixels[offset + 2] = (byte)(tile / (_tilesPerAxis * _tilesPerAxis));
            _indirectionPixels[offset + 3] = byte.MaxValue;
        }
        else
        {
            _indirectionPixels[offset] = 0;
            _indirectionPixels[offset + 1] = 0;
            _indirectionPixels[offset + 2] = 0;
            _indirectionPixels[offset + 3] = state == SparseSdfVolume.Air ? (byte)0 : (byte)128;
        }
    }

    private unsafe void UploadAtlasTile(SparseSdfVolume volume, int brick, int tile)
    {
        Span<ushort> pixels = stackalloc ushort[9 * 9 * 9];
        int brickX = brick % volume.BricksX;
        int brickY = brick / volume.BricksX % volume.BricksY;
        int brickZ = brick / (volume.BricksX * volume.BricksY);
        int index = 0;
        for (int z = 0; z < 9; z++)
        {
            for (int y = 0; y < 9; y++)
            {
                for (int x = 0; x < 9; x++)
                {
                    pixels[index++] = BitConverter.HalfToUInt16Bits(
                        (Half)volume.GetVoxel(brickX * 8 + x, brickY * 8 + y, brickZ * 8 + z));
                }
            }
        }

        int tileX = tile % _tilesPerAxis;
        int tileY = tile / _tilesPerAxis % _tilesPerAxis;
        int tileZ = tile / (_tilesPerAxis * _tilesPerAxis);
        fixed (ushort* pointer = pixels)
        {
            _gl.TexSubImage3D(
                TextureTarget.Texture3D,
                0,
                tileX * 9,
                tileY * 9,
                tileZ * 9,
                9,
                9,
                9,
                PixelFormat.Red,
                (PixelType)HalfFloat,
                pointer);
        }
    }

    private unsafe void UpdateRgba8Texture(uint texture, int width, int height, int depth, byte[] pixels)
    {
        _gl.BindTexture(TextureTarget.Texture3D, texture);
        fixed (byte* pointer = pixels)
        {
            _gl.TexSubImage3D(
                TextureTarget.Texture3D,
                0,
                0,
                0,
                0,
                (uint)width,
                (uint)height,
                (uint)depth,
                PixelFormat.Rgba,
                PixelType.UnsignedByte,
                pointer);
        }
        _gl.BindTexture(TextureTarget.Texture3D, 0);
    }

    private unsafe void UpdateCoarseTexture(SparseSdfVolume volume)
    {
        var coarse = new ushort[volume.Coarse.Length];
        for (int i = 0; i < coarse.Length; i++)
            coarse[i] = BitConverter.HalfToUInt16Bits((Half)volume.Coarse[i]);

        _gl.BindTexture(TextureTarget.Texture3D, _coarseTexture);
        fixed (ushort* pointer = coarse)
        {
            _gl.TexSubImage3D(
                TextureTarget.Texture3D,
                0,
                0,
                0,
                0,
                (uint)volume.CoarseX,
                (uint)volume.CoarseY,
                (uint)volume.CoarseZ,
                PixelFormat.Red,
                (PixelType)HalfFloat,
                pointer);
        }
        _gl.BindTexture(TextureTarget.Texture3D, 0);
    }

    private unsafe void InitializeFullscreenQuad()
    {
        float[] vertices = [-1f, -1f, 1f, -1f, 1f, 1f, -1f, -1f, 1f, 1f, -1f, 1f];
        _vao = _gl.GenVertexArray();
        _vbo = _gl.GenBuffer();
        _gl.BindVertexArray(_vao);
        _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
        fixed (float* pointer = vertices)
        {
            _gl.BufferData(
                BufferTargetARB.ArrayBuffer,
                (nuint)(vertices.Length * sizeof(float)),
                pointer,
                BufferUsageARB.StaticDraw);
        }
        _gl.EnableVertexAttribArray(0);
        _gl.VertexAttribPointer(0, 2, VertexAttribPointerType.Float, false, 2 * sizeof(float), null);
        _gl.BindVertexArray(0);
        _gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);
    }

    private unsafe void UploadR16fTexture(
        ref uint texture,
        int width,
        int height,
        int depth,
        ushort[] pixels,
        bool linear)
    {
        if (texture == 0)
            texture = _gl.GenTexture();

        _gl.BindTexture(TextureTarget.Texture3D, texture);
        SetTextureParameters(linear);
        fixed (ushort* pointer = pixels)
        {
            _gl.TexImage3D(
                TextureTarget.Texture3D,
                0,
                (InternalFormat)R16f,
                (uint)width,
                (uint)height,
                (uint)depth,
                0,
                PixelFormat.Red,
                (PixelType)HalfFloat,
                pointer);
        }
        _gl.BindTexture(TextureTarget.Texture3D, 0);
    }

    private unsafe void UploadRgba8Texture(
        ref uint texture,
        int width,
        int height,
        int depth,
        byte[] pixels)
    {
        if (texture == 0)
            texture = _gl.GenTexture();

        _gl.BindTexture(TextureTarget.Texture3D, texture);
        SetTextureParameters(linear: false);
        fixed (byte* pointer = pixels)
        {
            _gl.TexImage3D(
                TextureTarget.Texture3D,
                0,
                InternalFormat.Rgba8,
                (uint)width,
                (uint)height,
                (uint)depth,
                0,
                PixelFormat.Rgba,
                PixelType.UnsignedByte,
                pointer);
        }
        _gl.BindTexture(TextureTarget.Texture3D, 0);
    }

    private void SetTextureParameters(bool linear)
    {
        int filter = (int)(linear ? TextureMinFilter.Linear : TextureMinFilter.Nearest);
        _gl.TexParameter(TextureTarget.Texture3D, TextureParameterName.TextureMinFilter, filter);
        _gl.TexParameter(TextureTarget.Texture3D, TextureParameterName.TextureMagFilter, filter);
        _gl.TexParameter(TextureTarget.Texture3D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
        _gl.TexParameter(TextureTarget.Texture3D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToEdge);
        _gl.TexParameter(TextureTarget.Texture3D, TextureParameterName.TextureWrapR, (int)TextureWrapMode.ClampToEdge);
    }

    private void InitializeProgram()
    {
        _program = CreateProgram(VertexShaderSource, FragmentShaderSource);
        _inverseViewProjectionLocation = _gl.GetUniformLocation(_program, "uInverseViewProjection");
        _viewProjectionLocation = _gl.GetUniformLocation(_program, "uViewProjection");
        _volumeToWorldLocation = _gl.GetUniformLocation(_program, "uVolumeToWorld");
        _worldToVolumeLocation = _gl.GetUniformLocation(_program, "uWorldToVolume");
        _materialColorLocation = _gl.GetUniformLocation(_program, "uMaterialColor");
        _lightDirectionLocation = _gl.GetUniformLocation(_program, "uLightDirection");
        _atlasLocation = _gl.GetUniformLocation(_program, "uAtlas");
        _indirectionLocation = _gl.GetUniformLocation(_program, "uIndirection");
        _coarseLocation = _gl.GetUniformLocation(_program, "uCoarse");
        _stockLocation = _gl.GetUniformLocation(_program, "uStock");
        _volumeLocation = _gl.GetUniformLocation(_program, "uVolume");
        _voxelLocation = _gl.GetUniformLocation(_program, "uVoxel");
        _bandLocation = _gl.GetUniformLocation(_program, "uBand");
        _brickLocation = _gl.GetUniformLocation(_program, "uBrick");
        _atlasTexelsLocation = _gl.GetUniformLocation(_program, "uAtlasTexels");
        _resolutionLocation = _gl.GetUniformLocation(_program, "uResolution");
        _debugViewLocation = _gl.GetUniformLocation(_program, "uDebugView");
    }

    private uint CreateProgram(string vertexSource, string fragmentSource)
    {
        uint vertexShader = CompileShader(ShaderType.VertexShader, vertexSource);
        uint fragmentShader = CompileShader(ShaderType.FragmentShader, fragmentSource);
        uint program = _gl.CreateProgram();
        _gl.AttachShader(program, vertexShader);
        _gl.AttachShader(program, fragmentShader);
        _gl.LinkProgram(program);
        _gl.GetProgram(program, ProgramPropertyARB.LinkStatus, out int linked);
        _gl.DeleteShader(vertexShader);
        _gl.DeleteShader(fragmentShader);

        if (linked == 0)
        {
            string error = _gl.GetProgramInfoLog(program);
            _gl.DeleteProgram(program);
            throw new InvalidOperationException($"RW.CSG raymarch shader link failed: {error}");
        }

        return program;
    }

    private uint CompileShader(ShaderType type, string source)
    {
        uint shader = _gl.CreateShader(type);
        _gl.ShaderSource(shader, source);
        _gl.CompileShader(shader);
        _gl.GetShader(shader, ShaderParameterName.CompileStatus, out int compiled);
        if (compiled != 0)
            return shader;

        string error = _gl.GetShaderInfoLog(shader);
        _gl.DeleteShader(shader);
        throw new InvalidOperationException($"RW.CSG {type} shader compilation failed: {error}");
    }

    private const string VertexShaderSource = """
#version 330 core
layout(location = 0) in vec2 aPosition;

void main()
{
    gl_Position = vec4(aPosition, 0.0, 1.0);
}
""";

    private const string FragmentShaderSource = """
#version 330 core

layout(location = 0) out vec4 FragColor;

uniform sampler3D uAtlas;
uniform sampler3D uIndirection;
uniform sampler3D uCoarse;

uniform mat4 uInverseViewProjection;
uniform mat4 uViewProjection;
uniform mat4 uVolumeToWorld;
uniform mat4 uWorldToVolume;
uniform vec4 uMaterialColor;
uniform vec3 uLightDirection;
uniform vec3 uStock;
uniform vec3 uVolume;
uniform float uVoxel;
uniform float uBand;
uniform float uBrick;
uniform float uAtlasTexels;
uniform vec2 uResolution;
uniform int uDebugView;

float stockAt(vec3 p)
{
    vec3 q = abs(p - 0.5 * uStock) - 0.5 * uStock;
    return length(max(q, vec3(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
}

float voxelAt(ivec3 voxel)
{
    ivec3 voxelCount = ivec3(floor(uVolume / uVoxel + 0.5));
    ivec3 vi = clamp(voxel, ivec3(0), voxelCount - ivec3(1));
    ivec3 brick = vi / 8;
    vec4 indirection = texelFetch(uIndirection, brick, 0);

    if (indirection.a < 0.25)
        return uBand;
    if (indirection.a < 0.75)
        return -uBand;

    ivec3 tile = ivec3(floor(indirection.rgb * 255.0 + 0.5));
    ivec3 local = vi - brick * 8;
    return texelFetch(uAtlas, tile * 9 + local, 0).r;
}

float rawFieldAt(vec3 p)
{
    vec3 q = clamp(p, vec3(0.5 * uVoxel), uVolume - vec3(0.5 * uVoxel));
    vec3 voxelGrid = q / uVoxel - 0.5;
    ivec3 cell = ivec3(floor(voxelGrid * 0.125));
    vec3 local = voxelGrid - vec3(cell) * 8.0;
    vec4 indirection = texelFetch(uIndirection, cell, 0);
    float sampled;

    if (indirection.a < 0.25)
    {
        sampled = uBand;
    }
    else if (indirection.a < 0.75)
    {
        sampled = -uBand;
    }
    else
    {
        vec3 tile = floor(indirection.rgb * 255.0 + 0.5);
        vec3 atlasCoordinate = (tile * 9.0 + local + 0.5) / uAtlasTexels;
        sampled = texture(uAtlas, atlasCoordinate).r;
    }

    if (indirection.a < 0.75 && any(greaterThanEqual(local, vec3(7.0))))
    {
        ivec3 baseVoxel = ivec3(floor(voxelGrid));
        vec3 fraction = fract(voxelGrid);
        float c000 = voxelAt(baseVoxel);
        float c100 = voxelAt(baseVoxel + ivec3(1, 0, 0));
        float c010 = voxelAt(baseVoxel + ivec3(0, 1, 0));
        float c110 = voxelAt(baseVoxel + ivec3(1, 1, 0));
        float c001 = voxelAt(baseVoxel + ivec3(0, 0, 1));
        float c101 = voxelAt(baseVoxel + ivec3(1, 0, 1));
        float c011 = voxelAt(baseVoxel + ivec3(0, 1, 1));
        float c111 = voxelAt(baseVoxel + ivec3(1, 1, 1));
        sampled = mix(
            mix(mix(c000, c100, fraction.x), mix(c010, c110, fraction.x), fraction.y),
            mix(mix(c001, c101, fraction.x), mix(c011, c111, fraction.x), fraction.y),
            fraction.z);
    }

    return sampled;
}

float fieldAt(vec3 p)
{
    return max(rawFieldAt(p), stockAt(p));
}

// A forced minimum ray step can cross an entire thin surface at a grazing angle.
// The stored field is trilinear within each voxel cell, hence cubic along a ray.
// Split the short step at voxel planes and inspect its derivative extrema before
// advancing. Only a confirmed negative field sample supplies a crossing bracket;
// a small positive distance alone never invents a surface.
bool firstRawCrossing(vec3 origin, vec3 direction, float start, float end,
    out float outsideT, out float insideT)
{
    outsideT = start;
    insideT = start;
    float current = start;
    // A step is at most 0.35 voxel, crossing at most one plane per axis.
    for (int piece = 0; piece < 5 && current < end; piece++)
    {
        vec3 p = origin + direction * current;
        float boundary = end;
        for (int axis = 0; axis < 3; axis++)
        {
            float d = direction[axis];
            if (abs(d) < 1e-7)
                continue;
            float grid = p[axis] / uVoxel - 0.5;
            float plane = d > 0.0 ? floor(grid) + 1.0 : ceil(grid) - 1.0;
            float crossing = ((plane + 0.5) * uVoxel - origin[axis]) / d;
            if (crossing <= current + 1e-6)
                crossing += uVoxel / abs(d);
            boundary = min(boundary, crossing);
        }

        float span = boundary - current;
        float f0 = rawFieldAt(origin + direction * current);
        float f1 = rawFieldAt(origin + direction * (current + span / 3.0));
        float f2 = rawFieldAt(origin + direction * (current + 2.0 * span / 3.0));
        float f3 = rawFieldAt(origin + direction * boundary);
        float d1 = f1 - f0;
        float d2 = f2 - 2.0 * f1 + f0;
        float d3 = f3 - 3.0 * f2 + 3.0 * f1 - f0;
        float c1 = 3.0 * d1 - 1.5 * d2 + d3;
        float c2 = 4.5 * (d2 - d3);
        float c3 = 4.5 * d3;
        float a = 3.0 * c3;
        float b = 2.0 * c2;
        float knots[4];
        knots[0] = 0.0;
        knots[1] = 1.0;
        int knotCount = 2;
        if (abs(a) < 1e-7)
        {
            if (abs(b) > 1e-7)
            {
                float root = -c1 / b;
                if (root > 0.0 && root < 1.0)
                    knots[knotCount++] = root;
            }
        }
        else
        {
            float discriminant = b * b - 4.0 * a * c1;
            if (discriminant >= 0.0)
            {
                float squareRoot = sqrt(discriminant);
                float q = -0.5 * (b + (b >= 0.0 ? squareRoot : -squareRoot));
                float root1 = q / a;
                float root2 = q != 0.0 ? c1 / q : -1.0;
                if (root1 > 0.0 && root1 < 1.0)
                    knots[knotCount++] = root1;
                if (root2 > 0.0 && root2 < 1.0)
                    knots[knotCount++] = root2;
            }
        }
        for (int i = 1; i < knotCount; i++)
            for (int j = i; j > 0; j--)
                if (knots[j] < knots[j - 1])
                {
                    float temporary = knots[j];
                    knots[j] = knots[j - 1];
                    knots[j - 1] = temporary;
                }
        for (int i = 1; i < knotCount; i++)
        {
            float candidate = current + span * knots[i];
            if (rawFieldAt(origin + direction * candidate) < 0.0)
            {
                outsideT = current + span * knots[i - 1];
                insideT = candidate;
                return true;
            }
        }
        current = boundary;
    }
    return false;
}

float surfaceAt(vec3 p)
{
    float radius = 0.38 * uVoxel;
    float tetrahedralAverage = 0.25 * (
        fieldAt(p + vec3( radius,  radius,  radius)) +
        fieldAt(p + vec3( radius, -radius, -radius)) +
        fieldAt(p + vec3(-radius,  radius, -radius)) +
        fieldAt(p + vec3(-radius, -radius,  radius)));
    return mix(fieldAt(p), tetrahedralAverage, 0.65);
}

bool isUniformSolidAt(vec3 p)
{
    vec3 q = clamp(p, vec3(0.5 * uVoxel), uVolume - vec3(0.5 * uVoxel));
    vec3 voxelGrid = q / uVoxel - 0.5;
    ivec3 cell = ivec3(floor(voxelGrid * 0.125));
    float state = texelFetch(uIndirection, cell, 0).a;
    return state >= 0.25 && state < 0.75;
}

vec3 heatColor(float value)
{
    float x = clamp(value, 0.0, 1.0);
    return clamp(vec3(
        1.5 - abs(4.0 * x - 3.0),
        1.5 - abs(4.0 * x - 2.0),
        1.5 - abs(4.0 * x - 1.0)), 0.0, 1.0);
}

bool intersectBox(vec3 origin, vec3 direction, vec3 bounds, out float t0, out float t1)
{
    vec3 safeDirection = vec3(
        abs(direction.x) < 1e-6 ? (direction.x < 0.0 ? -1e-6 : 1e-6) : direction.x,
        abs(direction.y) < 1e-6 ? (direction.y < 0.0 ? -1e-6 : 1e-6) : direction.y,
        abs(direction.z) < 1e-6 ? (direction.z < 0.0 ? -1e-6 : 1e-6) : direction.z);
    vec3 inverseDirection = 1.0 / safeDirection;
    vec3 ta = -origin * inverseDirection;
    vec3 tb = (bounds - origin) * inverseDirection;
    vec3 nearPlane = min(ta, tb);
    vec3 farPlane = max(ta, tb);
    t0 = max(max(nearPlane.x, nearPlane.y), nearPlane.z);
    t1 = min(min(farPlane.x, farPlane.y), farPlane.z);
    return t0 < t1;
}

void main()
{
    vec2 uv = gl_FragCoord.xy / uResolution;
    vec4 nearClip = vec4(uv * 2.0 - 1.0, -1.0, 1.0);
    vec4 farClip = vec4(uv * 2.0 - 1.0, 1.0, 1.0);
    vec4 nearWorld = uInverseViewProjection * nearClip;
    vec4 farWorld = uInverseViewProjection * farClip;
    nearWorld /= nearWorld.w;
    farWorld /= farWorld.w;

    vec3 worldOrigin = nearWorld.xyz;
    vec3 worldDirection = normalize(farWorld.xyz - nearWorld.xyz);
    vec3 origin = (uWorldToVolume * vec4(worldOrigin, 1.0)).xyz;
    vec3 direction = normalize((uWorldToVolume * vec4(worldDirection, 0.0)).xyz);

    float stockEntry;
    float stockExit;
    if (!intersectBox(origin, direction, uStock, stockEntry, stockExit))
        discard;

    float traceT = max(stockEntry, 0.0) + 0.0001;
    float previousT = traceT;
    bool hit = false;
    bool solidHit = false;
    bool grazingHit = false;
    bool filteredHit = false;
    bool atStockEntry = true;
    int filterReject = 0;
    int steps = 0;
    float closestDistance = uBand;
    float closestT = traceT;

    for (int step = 0; step < 384; step++)
    {
        if (traceT >= stockExit)
            break;

        steps++;
        vec3 point = origin + direction * traceT;
        float coarseDistance = texture(uCoarse, clamp(point / uVolume, vec3(0.0), vec3(1.0))).r;
        if (coarseDistance > uBrick)
        {
            previousT = traceT;
            traceT += max(coarseDistance - 0.6 * uBrick, 0.35 * uVoxel);
            continue;
        }

        float distance = fieldAt(point);
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestT = traceT;
        }
        if (distance < 0.0)
        {
            hit = true;
            solidHit = atStockEntry && isUniformSolidAt(point);
            break;
        }

        atStockEntry = false;
        float nextT = traceT + max(distance, 0.35 * uVoxel);
        if (distance < 0.35 * uVoxel)
        {
            float outsideT;
            float insideT;
            if (firstRawCrossing(origin, direction, traceT, min(nextT, stockExit),
                outsideT, insideT))
            {
                previousT = outsideT;
                traceT = insideT;
                hit = true;
                break;
            }
        }
        previousT = traceT;
        traceT = nextT;
    }

    if (!hit)
    {
        if (closestDistance < 0.18 * uVoxel)
        {
            hit = true;
            grazingHit = true;
            traceT = closestT;
        }
        else if ((uDebugView == 1 && closestDistance < 0.999 * uBand) || uDebugView == 2)
        {
            float diagnosticT = clamp(closestT, max(stockEntry, 0.0), stockExit);
            vec3 diagnosticPoint = origin + direction * diagnosticT;
            vec3 diagnosticWorldPoint = (uVolumeToWorld * vec4(diagnosticPoint, 1.0)).xyz;
            vec4 diagnosticClip = uViewProjection * vec4(diagnosticWorldPoint, 1.0);
            float diagnosticDepth = 0.5 * (diagnosticClip.z / diagnosticClip.w) + 0.5;
            if (diagnosticDepth < 0.0 || diagnosticDepth > 1.0)
                discard;

            gl_FragDepth = diagnosticDepth;
            if (uDebugView == 1)
            {
                float nearMiss = 1.0 - clamp(closestDistance / uBand, 0.0, 1.0);
                FragColor = vec4(mix(vec3(0.12), vec3(1.0, 0.05, 0.02), nearMiss), 1.0);
            }
            else
            {
                float cost = clamp(log2(float(steps) + 1.0) / 8.0, 0.0, 1.0);
                FragColor = vec4(0.42 * heatColor(cost), 1.0);
            }
            return;
        }
        else
        {
            discard;
        }
    }

    float hitT = traceT;
    if (!solidHit && !grazingHit && previousT < traceT)
    {
        float lo = previousT;
        float hi = traceT;
        for (int refinement = 0; refinement < 7; refinement++)
        {
            float mid = 0.5 * (lo + hi);
            if (fieldAt(origin + direction * mid) < 0.0)
                hi = mid;
            else
                lo = mid;
        }
        hitT = 0.5 * (lo + hi);

        float smoothLo = max(stockEntry, hitT - uVoxel);
        float smoothHi = min(stockExit, hitT + uVoxel);
        float smoothLoDistance = surfaceAt(origin + direction * smoothLo);
        float smoothHiDistance = surfaceAt(origin + direction * smoothHi);
        if (smoothLoDistance >= 0.0 && smoothHiDistance <= 0.0)
        {
            for (int refinement = 0; refinement < 6; refinement++)
            {
                float mid = 0.5 * (smoothLo + smoothHi);
                if (surfaceAt(origin + direction * mid) < 0.0)
                    smoothHi = mid;
                else
                    smoothLo = mid;
            }
            hitT = 0.5 * (smoothLo + smoothHi);
            filteredHit = true;
        }
        else
        {
            if (smoothLoDistance < 0.0)
                filterReject += 1;
            if (smoothHiDistance > 0.0)
                filterReject += 2;
        }
    }

    vec3 hitPoint = origin + direction * hitT;
    float normalStep = max(1.15 * uVoxel, 0.02);
    vec3 gradient = solidHit
        ? vec3(
            stockAt(hitPoint + vec3(normalStep, 0.0, 0.0)) - stockAt(hitPoint - vec3(normalStep, 0.0, 0.0)),
            stockAt(hitPoint + vec3(0.0, normalStep, 0.0)) - stockAt(hitPoint - vec3(0.0, normalStep, 0.0)),
            stockAt(hitPoint + vec3(0.0, 0.0, normalStep)) - stockAt(hitPoint - vec3(0.0, 0.0, normalStep)))
        : vec3(
            surfaceAt(hitPoint + vec3(normalStep, 0.0, 0.0)) - surfaceAt(hitPoint - vec3(normalStep, 0.0, 0.0)),
            surfaceAt(hitPoint + vec3(0.0, normalStep, 0.0)) - surfaceAt(hitPoint - vec3(0.0, normalStep, 0.0)),
            surfaceAt(hitPoint + vec3(0.0, 0.0, normalStep)) - surfaceAt(hitPoint - vec3(0.0, 0.0, normalStep)));
    vec3 normal = length(gradient) > 1e-6 ? normalize(gradient) : -direction;
    vec3 color;

    if (uDebugView == 1)
    {
        color = solidHit ? vec3(0.10, 0.38, 1.00)
            : grazingHit ? vec3(1.00, 0.00, 0.85)
            : length(gradient) <= 1e-6 ? vec3(0.00, 1.00, 1.00)
            : filteredHit ? vec3(0.05, 0.95, 0.30)
            : filterReject == 1 ? vec3(1.00, 0.35, 0.00)
            : filterReject == 2 ? vec3(1.00, 0.78, 0.00)
            : vec3(1.00);
    }
    else if (uDebugView == 2)
    {
        float cost = clamp(log2(float(steps) + 1.0) / 8.0, 0.0, 1.0);
        color = heatColor(cost);
    }
    else if (uDebugView == 3)
    {
        vec3 brickCoordinate = (hitPoint / uVoxel - 0.5) * 0.125;
        vec3 brickId = floor(brickCoordinate);
        vec3 idColor = 0.25 + 0.65 * vec3(
            fract(sin(dot(brickId, vec3(127.1, 311.7,  74.7))) * 43758.5453),
            fract(sin(dot(brickId, vec3(269.5, 183.3, 246.1))) * 43758.5453),
            fract(sin(dot(brickId, vec3(113.5, 271.9, 124.6))) * 43758.5453));
        vec3 phase = fract(brickCoordinate);
        vec3 edgeDistance = min(phase, 1.0 - phase);
        float edge = min(edgeDistance.x, min(edgeDistance.y, edgeDistance.z));
        float line = 1.0 - smoothstep(0.025, 0.09, edge);
        color = mix(idColor, vec3(1.0), line);
    }
    else if (uDebugView == 4)
    {
        vec3 phase = fract(hitPoint / uVoxel - 0.5);
        vec3 edgeDistance = min(phase, 1.0 - phase);
        float edge = min(edgeDistance.x, min(edgeDistance.y, edgeDistance.z));
        float line = 1.0 - smoothstep(0.04, 0.12, edge);
        color = mix(0.12 + 0.76 * phase, vec3(1.0), line);
    }
    else if (uDebugView == 5)
    {
        float residual = clamp(fieldAt(hitPoint) / (0.5 * uVoxel), -1.0, 1.0);
        vec3 zeroColor = vec3(0.92);
        color = residual < 0.0
            ? mix(zeroColor, vec3(0.03, 0.22, 1.0), -residual)
            : mix(zeroColor, vec3(1.0, 0.08, 0.02), residual);
    }
    else if (uDebugView == 6)
    {
        color = 0.5 + 0.5 * normal;
    }
    else
    {
        vec3 light = normalize(uLightDirection);
        vec3 viewDirection = normalize(-direction);
        float diffuse = max(dot(normal, light), 0.0);
        float specular = pow(max(dot(normal, normalize(light + viewDirection)), 0.0), 28.0);
        color = uMaterialColor.rgb * (0.30 + 0.70 * diffuse) + vec3(0.18) * specular;
    }

    vec3 worldHitPoint = (uVolumeToWorld * vec4(hitPoint, 1.0)).xyz;
    vec4 clip = uViewProjection * vec4(worldHitPoint, 1.0);
    float depth = 0.5 * (clip.z / clip.w) + 0.5;
    if (depth < 0.0 || depth > 1.0)
        discard;

    gl_FragDepth = depth;
    FragColor = vec4(color, uMaterialColor.a);
}
""";
}

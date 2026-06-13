using System;
using Microsoft.Kinect.Fusion;

namespace KinectScanner
{
    /// <summary>
    /// A reconstruction volume preset. The volume is the 3D box (in front of the
    /// camera) inside which geometry is fused. Detail = VoxelsPerMeter; physical
    /// size of each axis = Voxels / VoxelsPerMeter.
    /// </summary>
    public class VolumePreset
    {
        public string Name;
        public float VoxelsPerMeter;
        public int VoxelsX;
        public int VoxelsY;
        public int VoxelsZ;
        public float DefaultMinDepth;
        public float DefaultMaxDepth;
        public string Tip;

        public float SizeX { get { return VoxelsX / VoxelsPerMeter; } }
        public float SizeY { get { return VoxelsY / VoxelsPerMeter; } }
        public float SizeZ { get { return VoxelsZ / VoxelsPerMeter; } }

        public override string ToString()
        {
            return string.Format("{0}  ({1:0.0}×{2:0.0}×{3:0.0} m @ {4:0}/m)",
                Name, SizeX, SizeY, SizeZ, VoxelsPerMeter);
        }

        public static VolumePreset[] All = new[]
        {
            // Bust is the most motion-sensitive mode (close range amplifies movement,
            // and a head has little geometry to track). The volume is sized to take in
            // the shoulders/upper chest — feature-rich geometry that anchors tracking —
            // while keeping ~2.6 mm detail on the face. Depth window starts at 0.6 m to
            // stay off the noisy 0.5 m sensor floor.
            new VolumePreset { Name = "Bust – head & shoulders (high detail)",
                VoxelsPerMeter = 384, VoxelsX = 448, VoxelsY = 448, VoxelsZ = 448,
                DefaultMinDepth = 0.6f, DefaultMaxDepth = 1.5f,
                Tip = "Highest detail, but the most motion-sensitive. Rotate VERY slowly "
                    + "(≈2 min/turn) and keep your shoulders in frame — a bare head is hard to track." },
            new VolumePreset { Name = "Seated person (swivel chair)",
                VoxelsPerMeter = 256, VoxelsX = 384, VoxelsY = 384, VoxelsZ = 384,
                DefaultMinDepth = 0.7f, DefaultMaxDepth = 2.0f,
                Tip = "Good all-rounder. Rotate smoothly, ~1 min/turn, keeping arms still." },
            new VolumePreset { Name = "Full body, standing",
                VoxelsPerMeter = 256, VoxelsX = 384, VoxelsY = 512, VoxelsZ = 384,
                DefaultMinDepth = 1.4f, DefaultMaxDepth = 2.8f,
                Tip = "Most forgiving for tracking (far range). Hold a still pose and turn slowly." },
        };
    }

    /// <summary>
    /// Wraps the Kinect Fusion color reconstruction volume. All methods must be
    /// called under the caller's lock — this class is not thread-safe by itself.
    /// </summary>
    public sealed class ScanEngine : IDisposable
    {
        public const int DepthWidth = 512;
        public const int DepthHeight = 424;
        private const int PixelCount = DepthWidth * DepthHeight;

        // --- Auto-recovery (camera pose finder / relocalization) tuning ---
        // Attempt relocalization once tracking has failed this many frames in a row.
        private const int RecoverAfterFailures = 3;
        // Try at most this many candidate poses from the pose-finder database.
        private const int MaxPosesToTest = 5;
        // Iterations used to verify/refine a candidate pose against the reconstruction.
        private const int RecoveryAlignIterations = 7;
        // Accept a recovered pose only if its alignment energy is below this
        // (lower = better fit; matches the KinectFusionExplorer sample threshold).
        private const float MaxRecoveryAlignEnergy = 0.27f;
        // Feed the pose-finder database every Nth successfully tracked frame.
        private const int PoseFinderAddInterval = 3;
        // Minimum normalized distance between stored poses (the finder keeps frames spread out).
        private const float PoseFinderMinDistanceThreshold = 0.3f;

        // --- Tracking robustness ---
        // Bilateral depth smoothing reduces sensor noise so ICP stays locked on
        // marginal surfaces (mesh / dark chair backs, flat panels). From the
        // KinectFusionExplorer sample.
        private const int SmoothingKernelWidth = 1;             // 1 => 3x3 neighborhood
        private const float SmoothingDistanceThreshold = 0.04f; // meters
        // A couple more ICP iterations than the SDK default (7) for steadier tracking.
        private const int AlignIterationCount = 9;
        // Below this fraction of in-range depth pixels the view is too feature-poor to
        // blame the user for a tracking failure (e.g. a dark mesh chair back).
        private const float SparseViewFraction = 0.05f;

        private ColorReconstruction volume;
        private FusionFloatImageFrame depthFloatFrame;
        private FusionFloatImageFrame smoothDepthFloatFrame;
        private FusionFloatImageFrame activeDepthFrame; // depthFloat or smoothed, per frame
        private FusionColorImageFrame colorInputFrame;
        private FusionPointCloudImageFrame raycastPointCloud;
        private FusionColorImageFrame shadedSurfaceFrame;
        private FusionColorImageFrame shadedNormalsFrame;
        private FusionColorImageFrame raycastColorFrame;

        // Relocalization. The pose finder is fed a depth-derived grayscale image
        // (not RGB) so it works identically whether or not color capture is on.
        private CameraPoseFinder poseFinder;
        private FusionColorImageFrame poseFinderColorFrame;
        private int[] poseFinderColorPixels;
        private float[] depthFloatScratch;
        private int addPoseCounter;

        private Matrix4 worldToCamera = Matrix4.Identity;
        private Matrix4 defaultWorldToVolume;
        private VolumePreset preset;

        public bool UsingGpu { get; private set; }
        public string ProcessorDescription { get; private set; }
        public int FramesIntegrated { get; private set; }
        public int ConsecutiveFailures { get; private set; }
        public float LastAlignmentEnergy { get; private set; }
        public bool ColorCaptured { get; private set; }
        public bool VolumeReady { get { return volume != null; } }

        /// <summary>When true, lost tracking is relocalized automatically.</summary>
        public bool AutoRecoveryEnabled { get; set; } = true;
        /// <summary>True for the single frame on which tracking was just recovered.</summary>
        public bool JustRecovered { get; private set; }
        /// <summary>Number of keyframe poses currently stored for relocalization.</summary>
        public int StoredPoseCount { get; private set; }

        /// <summary>Bilateral-smooth depth before tracking for steadier alignment.</summary>
        public bool SmoothDepth { get; set; } = true;
        /// <summary>Fraction of the frame with usable in-range depth (0..1).</summary>
        public float ValidDepthFraction { get; private set; }
        /// <summary>True when too little of the frame has depth to fairly expect tracking.</summary>
        public bool SparseDepthView { get; private set; }

        /// <summary>Create (or re-create) the reconstruction volume for a preset.</summary>
        public void Recreate(VolumePreset newPreset, float minDepth)
        {
            DisposeVolume();
            preset = newPreset;

            var parameters = new ReconstructionParameters(
                newPreset.VoxelsPerMeter, newPreset.VoxelsX, newPreset.VoxelsY, newPreset.VoxelsZ);

            try
            {
                volume = ColorReconstruction.FusionCreateReconstruction(
                    parameters, ReconstructionProcessor.Amp, -1, Matrix4.Identity);
                UsingGpu = true;
                ProcessorDescription = DescribeDevice(ReconstructionProcessor.Amp);
            }
            catch (Exception)
            {
                // No DirectX 11 capable GPU available — fall back to CPU (much slower).
                volume = ColorReconstruction.FusionCreateReconstruction(
                    parameters, ReconstructionProcessor.Cpu, -1, Matrix4.Identity);
                UsingGpu = false;
                ProcessorDescription = "CPU (no DX11 GPU found — slow!)";
            }

            defaultWorldToVolume = volume.GetCurrentWorldToVolumeTransform();

            if (depthFloatFrame == null)
            {
                depthFloatFrame = new FusionFloatImageFrame(DepthWidth, DepthHeight);
                smoothDepthFloatFrame = new FusionFloatImageFrame(DepthWidth, DepthHeight);
                colorInputFrame = new FusionColorImageFrame(DepthWidth, DepthHeight);
                raycastPointCloud = new FusionPointCloudImageFrame(DepthWidth, DepthHeight);
                shadedSurfaceFrame = new FusionColorImageFrame(DepthWidth, DepthHeight);
                shadedNormalsFrame = new FusionColorImageFrame(DepthWidth, DepthHeight);
                raycastColorFrame = new FusionColorImageFrame(DepthWidth, DepthHeight);
                poseFinderColorFrame = new FusionColorImageFrame(DepthWidth, DepthHeight);
                poseFinderColorPixels = new int[PixelCount];
                depthFloatScratch = new float[PixelCount];
            }

            // The pose finder is best-effort: if creation fails the scan still works,
            // it just falls back to manual recovery.
            try
            {
                if (poseFinder == null)
                {
                    poseFinder = CameraPoseFinder.FusionCreateCameraPoseFinder(
                        CameraPoseFinderParameters.Defaults);
                }
            }
            catch (Exception)
            {
                poseFinder = null;
            }

            Reset(minDepth);
        }

        private static string DescribeDevice(ReconstructionProcessor type)
        {
            try
            {
                string description = string.Empty, instancePath = string.Empty;
                int memoryKB = 0;
                FusionDepthProcessor.GetDeviceInfo(type, -1, out description, out instancePath, out memoryKB);
                return "GPU: " + description;
            }
            catch (Exception)
            {
                return "GPU";
            }
        }

        /// <summary>
        /// Clear the volume and place it so its front face starts at minDepth in
        /// front of the camera (centered on the camera axis in X and Y).
        /// </summary>
        public void Reset(float minDepth)
        {
            if (volume == null)
            {
                return;
            }

            worldToCamera = Matrix4.Identity;
            Matrix4 worldToVolume = defaultWorldToVolume;
            worldToVolume.M43 -= minDepth * preset.VoxelsPerMeter;
            volume.ResetReconstruction(worldToCamera, worldToVolume);

            if (poseFinder != null)
            {
                try { poseFinder.ResetCameraPoseFinder(); }
                catch (Exception) { }
            }

            FramesIntegrated = 0;
            ConsecutiveFailures = 0;
            LastAlignmentEnergy = 0f;
            ColorCaptured = false;
            JustRecovered = false;
            StoredPoseCount = 0;
            addPoseCounter = 0;
        }

        /// <summary>
        /// Track the camera pose against the reconstruction and fuse the new frame in.
        /// Returns true when tracking succeeded. Always renders the current view of
        /// the reconstruction into <paramref name="displayPixels"/> (BGRA).
        /// </summary>
        public bool ProcessFrame(
            ushort[] depthData,
            int[] colorAtDepth,
            float minDepth,
            float maxDepth,
            int integrationWeight,
            bool renderColor,
            int[] displayPixels)
        {
            volume.DepthToDepthFloatFrame(depthData, depthFloatFrame, minDepth, maxDepth, false);
            ComputeDepthCoverage(depthData, minDepth, maxDepth);
            JustRecovered = false;

            // Bilateral-smooth the depth so ICP has a cleaner surface to lock onto.
            // The smoothed frame is used for alignment, integration and relocalization.
            if (SmoothDepth)
            {
                volume.SmoothDepthFloatFrame(
                    depthFloatFrame, smoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);
                activeDepthFrame = smoothDepthFloatFrame;
            }
            else
            {
                activeDepthFrame = depthFloatFrame;
            }

            // If tracking has been failing, try to relocalize from the pose-finder
            // database before integrating. On success we adopt the recovered pose but
            // skip integration for this frame; the next frame fuses from there.
            if (AutoRecoveryEnabled && poseFinder != null
                && ConsecutiveFailures >= RecoverAfterFailures
                && TryRelocalize())
            {
                JustRecovered = true;
                ConsecutiveFailures = 0;
                Render(renderColor, displayPixels);
                return true;
            }

            float alignmentEnergy;
            bool trackingOk;

            if (colorAtDepth != null)
            {
                colorInputFrame.CopyPixelDataFrom(colorAtDepth);
                trackingOk = volume.ProcessFrame(
                    activeDepthFrame,
                    colorInputFrame,
                    AlignIterationCount,
                    integrationWeight,
                    FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
                    out alignmentEnergy,
                    worldToCamera);
                ColorCaptured = true;
            }
            else
            {
                trackingOk = volume.ProcessFrame(
                    activeDepthFrame,
                    AlignIterationCount,
                    integrationWeight,
                    out alignmentEnergy,
                    worldToCamera);
            }

            LastAlignmentEnergy = alignmentEnergy;

            if (trackingOk)
            {
                worldToCamera = volume.GetCurrentWorldToCameraTransform();
                FramesIntegrated++;
                ConsecutiveFailures = 0;
                MaybeAddPoseToFinder();
            }
            else
            {
                ConsecutiveFailures++;
            }

            Render(renderColor, displayPixels);
            return trackingOk;
        }

        /// <summary>
        /// Query the pose-finder database for the current depth frame and, if a
        /// candidate aligns well to the reconstruction, adopt it as the camera pose.
        /// Best-effort: any failure returns false and leaves tracking unchanged.
        /// </summary>
        private bool TryRelocalize()
        {
            try
            {
                FillPoseFinderColorFromDepth();

                using (MatchCandidates candidates =
                    poseFinder.FindCameraPose(activeDepthFrame, poseFinderColorFrame))
                {
                    if (candidates == null || candidates.GetPoseCount() == 0)
                    {
                        return false;
                    }

                    var poses = candidates.GetMatchPoses();
                    float bestEnergy = float.MaxValue;
                    Matrix4 bestPose = worldToCamera;
                    bool found = false;

                    int tryCount = Math.Min(poses.Count, MaxPosesToTest);
                    for (int i = 0; i < tryCount; i++)
                    {
                        float energy;
                        bool aligned = volume.AlignDepthFloatToReconstruction(
                            activeDepthFrame, RecoveryAlignIterations, null, out energy, poses[i]);

                        if (aligned && energy < bestEnergy)
                        {
                            bestEnergy = energy;
                            bestPose = volume.GetCurrentWorldToCameraTransform();
                            found = true;
                        }
                    }

                    if (found && bestEnergy < MaxRecoveryAlignEnergy)
                    {
                        worldToCamera = bestPose;
                        LastAlignmentEnergy = bestEnergy;
                        return true;
                    }
                }
            }
            catch (Exception)
            {
                // Relocalization is optional — fall back to manual recovery.
            }

            return false;
        }

        /// <summary>Throttled: store the current frame as a relocalization keyframe.</summary>
        private void MaybeAddPoseToFinder()
        {
            if (poseFinder == null)
            {
                return;
            }

            if (++addPoseCounter % PoseFinderAddInterval != 0)
            {
                return;
            }

            try
            {
                FillPoseFinderColorFromDepth();
                bool addedPose, trimmedHistory;
                poseFinder.ProcessFrame(
                    activeDepthFrame, poseFinderColorFrame, worldToCamera,
                    PoseFinderMinDistanceThreshold, out addedPose, out trimmedHistory);
                if (addedPose)
                {
                    StoredPoseCount = poseFinder.GetStoredPoseCount();
                }
            }
            catch (Exception)
            {
                // Non-fatal: a missed keyframe just means slightly worse recovery odds.
            }
        }

        /// <summary>
        /// Fill the pose-finder color frame with a grayscale rendering of the current
        /// depth float frame, so relocalization is independent of RGB capture.
        /// </summary>
        /// <summary>Count usable in-range depth pixels to flag feature-poor views.</summary>
        private void ComputeDepthCoverage(ushort[] depthData, float minDepth, float maxDepth)
        {
            int lo = (int)(minDepth * 1000f);
            int hi = (int)(maxDepth * 1000f);
            int count = 0;
            int n = depthData.Length;
            for (int i = 0; i < n; i++)
            {
                int d = depthData[i];
                if (d >= lo && d <= hi)
                {
                    count++;
                }
            }

            ValidDepthFraction = (float)count / PixelCount;
            SparseDepthView = ValidDepthFraction < SparseViewFraction;
        }

        private void FillPoseFinderColorFromDepth()
        {
            activeDepthFrame.CopyPixelDataTo(depthFloatScratch);

            unchecked
            {
                const int opaque = (int)0xFF000000;
                for (int i = 0; i < PixelCount; i++)
                {
                    float d = depthFloatScratch[i];
                    int v;
                    if (d <= 0f)
                    {
                        v = 0;
                    }
                    else
                    {
                        // Map the 0.4–4.0 m working range onto 0–255.
                        float t = (d - 0.4f) / 3.6f;
                        if (t < 0f) t = 0f; else if (t > 1f) t = 1f;
                        v = (int)(t * 255f);
                    }
                    poseFinderColorPixels[i] = opaque | (v << 16) | (v << 8) | v;
                }
            }

            poseFinderColorFrame.CopyPixelDataFrom(poseFinderColorPixels);
        }

        /// <summary>Raycast the reconstruction from the current camera pose into a BGRA buffer.</summary>
        public void Render(bool renderColor, int[] displayPixels)
        {
            if (volume == null)
            {
                return;
            }

            if (renderColor)
            {
                volume.CalculatePointCloud(raycastPointCloud, raycastColorFrame, worldToCamera);
                raycastColorFrame.CopyPixelDataTo(displayPixels);
            }
            else
            {
                volume.CalculatePointCloud(raycastPointCloud, worldToCamera);
                FusionDepthProcessor.ShadePointCloud(
                    raycastPointCloud, worldToCamera, shadedSurfaceFrame, shadedNormalsFrame);
                shadedSurfaceFrame.CopyPixelDataTo(displayPixels);
            }
        }

        /// <summary>Extract the triangle mesh from the volume (vertices in meters).</summary>
        public ColorMesh CalculateMesh()
        {
            return volume.CalculateMesh(1);
        }

        private void DisposeVolume()
        {
            if (volume != null)
            {
                volume.Dispose();
                volume = null;
            }
        }

        public void Dispose()
        {
            DisposeVolume();
            if (depthFloatFrame != null) { depthFloatFrame.Dispose(); depthFloatFrame = null; }
            if (smoothDepthFloatFrame != null) { smoothDepthFloatFrame.Dispose(); smoothDepthFloatFrame = null; }
            if (colorInputFrame != null) { colorInputFrame.Dispose(); colorInputFrame = null; }
            if (raycastPointCloud != null) { raycastPointCloud.Dispose(); raycastPointCloud = null; }
            if (shadedSurfaceFrame != null) { shadedSurfaceFrame.Dispose(); shadedSurfaceFrame = null; }
            if (shadedNormalsFrame != null) { shadedNormalsFrame.Dispose(); shadedNormalsFrame = null; }
            if (raycastColorFrame != null) { raycastColorFrame.Dispose(); raycastColorFrame = null; }
            if (poseFinderColorFrame != null) { poseFinderColorFrame.Dispose(); poseFinderColorFrame = null; }
            if (poseFinder != null) { poseFinder.Dispose(); poseFinder = null; }
        }
    }
}

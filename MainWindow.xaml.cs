using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using Microsoft.Win32;

namespace KinectScanner
{
    public partial class MainWindow : Window
    {
        private enum ScanState { Idle, Scanning, Paused }

        private const int DepthWidth = ScanEngine.DepthWidth;
        private const int DepthHeight = ScanEngine.DepthHeight;
        private const int DepthPixelCount = DepthWidth * DepthHeight;
        private const int TrackingLostBannerThreshold = 12;
        // Fuse color only every Nth processed frame — geometry needs every frame,
        // color does not, and the 1080p conversion + mapping is the costliest CPU step.
        private const int ColorIntegrationInterval = 3;

        private KinectSensor sensor;
        private MultiSourceFrameReader reader;
        private CoordinateMapper mapper;

        private readonly ScanEngine engine = new ScanEngine();
        private readonly object engineLock = new object();
        private int workerBusy;   // 0 = free, 1 = fusion in flight (Interlocked)
        private int previewBusy;  // 0 = free, 1 = preview render in flight (Interlocked)
        private volatile bool shuttingDown;

        private ScanState state = ScanState.Idle;
        private bool initialized;
        private bool volumeReady;

        // Buffers filled on the UI thread from Kinect frames
        private ushort[] depthData;
        private byte[] colorBytes;
        private int colorWidth, colorHeight;

        // Snapshot buffers owned by the fusion worker (serialized by workerBusy)
        private ushort[] depthWork;
        private ColorSpacePoint[] colorSpacePoints;
        private int[] colorAtDepth;
        private int colorPhase;

        // Snapshot buffers owned by the preview worker (serialized by previewBusy)
        private ushort[] previewDepth;
        private int[] depthPreviewPixels;

        // Display — fusion output alternates buffers so the worker never waits on the UI
        private int[] displayBufferA;
        private int[] displayBufferB;
        private bool useBufferA;
        private WriteableBitmap reconstructionBitmap;
        private WriteableBitmap depthPreviewBitmap;

        // Diagnostics
        private DispatcherTimer statusTimer;
        private int fusedFpsCounter;
        private int depthFpsCounter;
        private int frameCounter;
        private int lastFusionMs;
        private int sensorDropouts;
        private bool sensorWasAvailable;

        // Audio cues — let the user scan by ear without watching the screen.
        private enum AudioCue { Good, Lost, Quiet }
        private SoundCues soundCues;
        private DispatcherTimer audioTimer;
        private AudioCue audioCue = AudioCue.Good;
        private DateTime lastHeartbeat;
        private DateTime lastAlert;
        // Consecutive tracking failures before the audio declares "lost" (hysteresis
        // so a single dropped frame doesn't buzz). Lower than the on-screen banner
        // threshold so the user hears trouble early.
        private const int AudioLostThreshold = 3;
        private const double HeartbeatIntervalMs = 2000;
        private const double AlertIntervalMs = 850;

        private static readonly object LogLock = new object();
        private static readonly string LogPath =
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "scanner.log");

        private void Log(string message)
        {
            try
            {
                lock (LogLock)
                {
                    File.AppendAllText(LogPath,
                        DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff  ") + message + Environment.NewLine);
                }
            }
            catch (Exception)
            {
                // Logging must never break the app
            }
        }

        public MainWindow()
        {
            InitializeComponent();
        }

        // ------------------------------------------------------------------
        // Startup / shutdown
        // ------------------------------------------------------------------

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            depthData = new ushort[DepthPixelCount];
            depthWork = new ushort[DepthPixelCount];
            previewDepth = new ushort[DepthPixelCount];
            colorSpacePoints = new ColorSpacePoint[DepthPixelCount];
            colorAtDepth = new int[DepthPixelCount];
            displayBufferA = new int[DepthPixelCount];
            displayBufferB = new int[DepthPixelCount];
            depthPreviewPixels = new int[DepthPixelCount];

            reconstructionBitmap = new WriteableBitmap(DepthWidth, DepthHeight, 96, 96, PixelFormats.Bgr32, null);
            depthPreviewBitmap = new WriteableBitmap(DepthWidth, DepthHeight, 96, 96, PixelFormats.Bgr32, null);
            MainImage.Source = depthPreviewBitmap;
            DepthPreviewImage.Source = depthPreviewBitmap;

            sensor = KinectSensor.GetDefault();
            mapper = sensor.CoordinateMapper;

            FrameDescription colorDesc = sensor.ColorFrameSource.FrameDescription;
            colorWidth = colorDesc.Width;
            colorHeight = colorDesc.Height;
            colorBytes = new byte[colorWidth * colorHeight * 4];

            OpenReader();
            sensor.IsAvailableChanged += Sensor_IsAvailableChanged;
            sensor.Open();
            Log("App started — session begins.");

            PresetCombo.ItemsSource = VolumePreset.All;
            initialized = true;
            PresetCombo.SelectedIndex = 1; // Seated person — triggers volume creation

            statusTimer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            statusTimer.Tick += StatusTimer_Tick;
            statusTimer.Start();

            try { soundCues = new SoundCues(); }
            catch (Exception) { soundCues = null; }

            audioTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(150) };
            audioTimer.Tick += AudioTimer_Tick;
            audioTimer.Start();

            UpdateUiState();
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            Log("App closing (dropouts this session: " + sensorDropouts + ").");
            shuttingDown = true;
            state = ScanState.Idle;

            if (reader != null)
            {
                reader.MultiSourceFrameArrived -= Reader_MultiSourceFrameArrived;
                reader.Dispose();
                reader = null;
            }

            // Let an in-flight Fusion frame finish before tearing the volume down
            var deadline = DateTime.UtcNow.AddSeconds(3);
            while (Interlocked.CompareExchange(ref workerBusy, 0, 0) == 1 && DateTime.UtcNow < deadline)
            {
                Thread.Sleep(20);
            }

            lock (engineLock)
            {
                engine.Dispose();
            }

            if (audioTimer != null)
            {
                audioTimer.Stop();
                audioTimer = null;
            }
            if (soundCues != null)
            {
                soundCues.Dispose();
                soundCues = null;
            }

            if (sensor != null)
            {
                sensor.Close();
                sensor = null;
            }
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (e.IsAvailable)
            {
                sensorWasAvailable = true;
                StatusText.Text = sensorDropouts > 0
                    ? string.Format("Kinect reconnected (dropouts so far: {0}).", sensorDropouts)
                    : "Kinect connected.";
                Log("Sensor AVAILABLE (state=" + state + ").");
            }
            else if (sensorWasAvailable)
            {
                sensorDropouts++;
                StatusText.Text = string.Format(
                    "Kinect connection dropped (#{0}) — see scanner.log next to the exe.", sensorDropouts);
                Log(string.Format("Sensor DROPOUT #{0} (state={1}, colorStream={2}).",
                    sensorDropouts, state, CaptureColorCheck.IsChecked == true));
            }
            else
            {
                StatusText.Text = "Kinect not detected — check USB 3.0 connection and power.";
            }
        }

        /// <summary>
        /// (Re)opens the frame reader, subscribing to the color stream only when
        /// color capture is enabled — the 1080p color stream is roughly half the
        /// Kinect's USB 3.0 bandwidth, so leaving it off eases marginal links.
        /// </summary>
        private void OpenReader()
        {
            if (reader != null)
            {
                reader.MultiSourceFrameArrived -= Reader_MultiSourceFrameArrived;
                reader.Dispose();
                reader = null;
            }

            FrameSourceTypes types = FrameSourceTypes.Depth;
            if (CaptureColorCheck.IsChecked == true)
            {
                types |= FrameSourceTypes.Color;
            }

            reader = sensor.OpenMultiSourceFrameReader(types);
            reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            Log("Reader opened (sources: " + types + ").");
        }

        // ------------------------------------------------------------------
        // Volume creation
        // ------------------------------------------------------------------

        private VolumePreset CurrentPreset
        {
            get { return PresetCombo.SelectedItem as VolumePreset; }
        }

        private void PresetCombo_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (!initialized || CurrentPreset == null)
            {
                return;
            }

            VolumePreset preset = CurrentPreset;
            MinDepthSlider.Value = preset.DefaultMinDepth;
            MaxDepthSlider.Value = preset.DefaultMaxDepth;
            PresetTip.Text = preset.Tip;
            RecreateVolumeAsync(preset);
        }

        private async void RecreateVolumeAsync(VolumePreset preset)
        {
            volumeReady = false;
            state = ScanState.Idle;
            UpdateUiState();
            StatusText.Text = "Allocating reconstruction volume…";

            float minDepth = (float)MinDepthSlider.Value;
            try
            {
                await Task.Run(() =>
                {
                    lock (engineLock)
                    {
                        engine.Recreate(preset, minDepth);
                    }
                });

                volumeReady = true;
                ProcessorText.Text = engine.ProcessorDescription;
                StatusText.Text = engine.UsingGpu
                    ? "Ready. Frame yourself in the depth view, then press Start Scan."
                    : "Ready (CPU mode — expect very low frame rates).";
                Log("Volume created: " + preset + " on " + engine.ProcessorDescription);
            }
            catch (Exception ex)
            {
                StatusText.Text = "Failed to create reconstruction volume: " + ex.Message;
            }

            UpdateUiState();
        }

        // ------------------------------------------------------------------
        // Frame pipeline
        // ------------------------------------------------------------------
        //
        // The UI-thread handler is kept as cheap as possible: one depth copy,
        // plus (only when a fusion worker is actually launched) one color
        // conversion. Everything else — preview shading, color mapping, fusion,
        // raycasting — happens on background threads, and display updates are
        // posted fire-and-forget so the workers never wait for WPF.

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            if (shuttingDown)
            {
                return;
            }

            MultiSourceFrame multiFrame = e.FrameReference.AcquireFrame();
            if (multiFrame == null)
            {
                return;
            }

            using (DepthFrame depthFrame = multiFrame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame == null)
                {
                    return;
                }
                depthFrame.CopyFrameDataToArray(depthData);
            }

            Interlocked.Increment(ref depthFpsCounter);
            frameCounter++;

            // Fusion — only if the previous frame is done; otherwise drop this one.
            if (state == ScanState.Scanning && volumeReady
                && Interlocked.CompareExchange(ref workerBusy, 1, 0) == 0)
            {
                bool useColor = false;
                if (CaptureColorCheck.IsChecked == true && colorPhase % ColorIntegrationInterval == 0)
                {
                    using (ColorFrame colorFrame = multiFrame.ColorFrameReference.AcquireFrame())
                    {
                        if (colorFrame != null)
                        {
                            colorFrame.CopyConvertedFrameDataToArray(colorBytes, ColorImageFormat.Bgra);
                            useColor = true;
                        }
                    }
                }
                colorPhase++;

                Array.Copy(depthData, depthWork, DepthPixelCount);
                float minDepth = (float)MinDepthSlider.Value;
                float maxDepth = (float)MaxDepthSlider.Value;
                int weight = (int)WeightSlider.Value;
                bool renderColor = CaptureColorCheck.IsChecked == true && ColorViewCheck.IsChecked == true;

                Task.Run(() => ProcessFrameWorker(useColor, minDepth, maxDepth, weight, renderColor));
            }

            // Depth preview — throttled, computed off-thread.
            int previewInterval = state == ScanState.Scanning ? 4 : 2;
            if (frameCounter % previewInterval == 0
                && Interlocked.CompareExchange(ref previewBusy, 1, 0) == 0)
            {
                Array.Copy(depthData, previewDepth, DepthPixelCount);
                float pMin = (float)MinDepthSlider.Value;
                float pMax = (float)MaxDepthSlider.Value;
                Task.Run(() => PreviewWorker(pMin, pMax));
            }
        }

        private void ProcessFrameWorker(bool useColor, float minDepth, float maxDepth, int weight, bool renderColor)
        {
            try
            {
                int[] colorInput = null;
                if (useColor)
                {
                    MapColorToDepth();
                    colorInput = colorAtDepth;
                }

                int[] display = useBufferA ? displayBufferA : displayBufferB;
                useBufferA = !useBufferA;

                bool trackingOk;
                int framesIntegrated, consecutiveFailures, storedPoses;
                bool justRecovered, sparseView;
                var stopwatch = Stopwatch.StartNew();
                lock (engineLock)
                {
                    if (shuttingDown || !engine.VolumeReady)
                    {
                        return;
                    }

                    trackingOk = engine.ProcessFrame(
                        depthWork, colorInput, minDepth, maxDepth, weight, renderColor && useColor, display);
                    framesIntegrated = engine.FramesIntegrated;
                    consecutiveFailures = engine.ConsecutiveFailures;
                    storedPoses = engine.StoredPoseCount;
                    justRecovered = engine.JustRecovered;
                    sparseView = engine.SparseDepthView;
                }
                stopwatch.Stop();
                lastFusionMs = (int)stopwatch.ElapsedMilliseconds;
                Interlocked.Increment(ref fusedFpsCounter);

                if (!shuttingDown)
                {
                    // Fire-and-forget: the fusion worker must never wait on WPF.
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        reconstructionBitmap.WritePixels(
                            new Int32Rect(0, 0, DepthWidth, DepthHeight),
                            display, DepthWidth * 4, 0);

                        FramesText.Text = "Frames fused: " + framesIntegrated.ToString("N0");

                        bool autoRecover = AutoRecoveryCheck.IsChecked == true;
                        bool lost = consecutiveFailures >= TrackingLostBannerThreshold;
                        TrackingBanner.Visibility = lost ? Visibility.Visible : Visibility.Collapsed;
                        TrackingBannerText.Text = autoRecover
                            ? "TRACKING LOST — hold still, recovering automatically… (or rotate back slowly)"
                            : "TRACKING LOST — slowly rotate back toward your last position (or press Reset)";

                        if (justRecovered)
                        {
                            TrackingDot.Fill = Brushes.DeepSkyBlue;
                            TrackingText.Text = "Recovered";
                        }
                        else if (trackingOk)
                        {
                            TrackingDot.Fill = Brushes.LimeGreen;
                            TrackingText.Text = "Tracking";
                        }
                        else if (sparseView)
                        {
                            // Too little depth to fairly expect tracking — not the user's fault.
                            TrackingDot.Fill = Brushes.SlateGray;
                            TrackingText.Text = "Sparse view — turn more of yourself into frame";
                            TrackingBanner.Visibility = Visibility.Collapsed;
                        }
                        else
                        {
                            bool recovering = autoRecover && storedPoses > 0;
                            TrackingDot.Fill = lost ? Brushes.Red : Brushes.Orange;
                            TrackingText.Text = recovering
                                ? "Recovering… (" + consecutiveFailures + ")"
                                : "Tracking lost (" + consecutiveFailures + ")";
                        }

                        UpdateAudioState(trackingOk, consecutiveFailures, sparseView);
                    }), DispatcherPriority.Render);
                }
            }
            catch (Exception ex)
            {
                Log("Processing error: " + ex.Message);
                if (!shuttingDown)
                {
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        StatusText.Text = "Processing error: " + ex.Message;
                    }));
                }
            }
            finally
            {
                Interlocked.Exchange(ref workerBusy, 0);
            }
        }

        /// <summary>Builds a 512×424 BGRA buffer of color values aligned to the depth pixels.</summary>
        private void MapColorToDepth()
        {
            mapper.MapDepthFrameToColorSpace(depthWork, colorSpacePoints);

            unchecked
            {
                const int opaque = (int)0xFF000000;
                for (int i = 0; i < DepthPixelCount; i++)
                {
                    ColorSpacePoint p = colorSpacePoints[i];
                    int cx = (int)(p.X + 0.5f);
                    int cy = (int)(p.Y + 0.5f);
                    if (cx >= 0 && cx < colorWidth && cy >= 0 && cy < colorHeight)
                    {
                        int o = (cy * colorWidth + cx) * 4;
                        colorAtDepth[i] = colorBytes[o] | (colorBytes[o + 1] << 8) | (colorBytes[o + 2] << 16) | opaque;
                    }
                    else
                    {
                        colorAtDepth[i] = 0;
                    }
                }
            }
        }

        private void PreviewWorker(float min, float max)
        {
            try
            {
                float range = Math.Max(0.05f, max - min);

                unchecked
                {
                    const int black = (int)0xFF000000;
                    const int outOfRange = (int)0xFF20242E;
                    for (int i = 0; i < DepthPixelCount; i++)
                    {
                        ushort raw = previewDepth[i];
                        if (raw == 0)
                        {
                            depthPreviewPixels[i] = black;
                            continue;
                        }

                        float meters = raw * 0.001f;
                        if (meters < min || meters > max)
                        {
                            depthPreviewPixels[i] = outOfRange;
                        }
                        else
                        {
                            int v = 250 - (int)((meters - min) / range * 190f);
                            depthPreviewPixels[i] = black | (v << 16) | (v << 8) | v;
                        }
                    }
                }

                if (shuttingDown)
                {
                    Interlocked.Exchange(ref previewBusy, 0);
                    return;
                }

                // previewBusy is released only after the pixels are on screen, so
                // this buffer is never written while WPF is reading it.
                Dispatcher.BeginInvoke(new Action(() =>
                {
                    try
                    {
                        depthPreviewBitmap.WritePixels(
                            new Int32Rect(0, 0, DepthWidth, DepthHeight),
                            depthPreviewPixels, DepthWidth * 4, 0);
                    }
                    finally
                    {
                        Interlocked.Exchange(ref previewBusy, 0);
                    }
                }), DispatcherPriority.Background);
            }
            catch (Exception)
            {
                Interlocked.Exchange(ref previewBusy, 0);
            }
        }

        // ------------------------------------------------------------------
        // Controls
        // ------------------------------------------------------------------

        private async void StartButton_Click(object sender, RoutedEventArgs e)
        {
            if (!volumeReady)
            {
                return;
            }

            StartButton.IsEnabled = false;
            float minDepth = (float)MinDepthSlider.Value;
            bool autoRecover = AutoRecoveryCheck.IsChecked == true;
            bool smoothDepth = SmoothDepthCheck.IsChecked == true;
            await Task.Run(() =>
            {
                lock (engineLock)
                {
                    engine.AutoRecoveryEnabled = autoRecover;
                    engine.SmoothDepth = smoothDepth;
                    engine.Reset(minDepth);
                }
            });

            colorPhase = 0;
            audioCue = AudioCue.Good;
            lastHeartbeat = DateTime.UtcNow;
            lastAlert = DateTime.UtcNow;
            state = ScanState.Scanning;
            MainImage.Source = reconstructionBitmap;
            StatusText.Text = "Scanning — hold still 2 s, then rotate very slowly.";
            Log("Scan started (depth window " + minDepth.ToString("0.00") + "–"
                + ((float)MaxDepthSlider.Value).ToString("0.00") + " m).");
            UpdateUiState();
        }

        private void PauseButton_Click(object sender, RoutedEventArgs e)
        {
            if (state == ScanState.Scanning)
            {
                state = ScanState.Paused;
                StatusText.Text = "Paused. Resume from (roughly) the same pose, or export the mesh.";
            }
            else if (state == ScanState.Paused)
            {
                state = ScanState.Scanning;
                StatusText.Text = "Scanning resumed.";
            }

            UpdateUiState();
        }

        private async void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            if (!volumeReady)
            {
                return;
            }

            ResetButton.IsEnabled = false;
            state = ScanState.Idle;
            float minDepth = (float)MinDepthSlider.Value;
            await Task.Run(() =>
            {
                lock (engineLock)
                {
                    engine.Reset(minDepth);
                }
            });

            MainImage.Source = depthPreviewBitmap;
            TrackingBanner.Visibility = Visibility.Collapsed;
            StatusText.Text = "Reconstruction cleared.";
            ResetButton.IsEnabled = true;
            UpdateUiState();
        }

        private async void ExportButton_Click(object sender, RoutedEventArgs e)
        {
            if (!volumeReady || engine.FramesIntegrated == 0)
            {
                StatusText.Text = "Nothing to export yet — run a scan first.";
                return;
            }

            if (state == ScanState.Scanning)
            {
                state = ScanState.Paused;
                UpdateUiState();
            }

            var dialog = new SaveFileDialog
            {
                Title = "Export 3D model",
                Filter = "PLY mesh with color (*.ply)|*.ply|Wavefront OBJ (*.obj)|*.obj|STL binary, no color (*.stl)|*.stl",
                FileName = "scan_" + DateTime.Now.ToString("yyyyMMdd_HHmmss"),
            };
            if (dialog.ShowDialog(this) != true)
            {
                return;
            }

            string path = dialog.FileName;
            bool withColor = engine.ColorCaptured;

            ExportButton.IsEnabled = false;
            StatusText.Text = "Generating mesh… (this can take a while)";
            try
            {
                var counts = await Task.Run(() =>
                {
                    ColorMesh mesh;
                    lock (engineLock)
                    {
                        mesh = engine.CalculateMesh();
                    }

                    using (mesh)
                    {
                        int vertexCount, triangleCount;
                        MeshExporter.Save(mesh, path, withColor, out vertexCount, out triangleCount);
                        return Tuple.Create(vertexCount, triangleCount);
                    }
                });

                StatusText.Text = string.Format("Exported {0:N0} vertices / {1:N0} triangles → {2}",
                    counts.Item1, counts.Item2, path);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, "Export failed:\n\n" + ex.Message, "Kinect 3D Scanner",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                StatusText.Text = "Export failed.";
            }
            finally
            {
                UpdateUiState();
            }
        }

        private void DepthSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!initialized)
            {
                return;
            }

            // Keep a sane gap between near and far clip
            if (MaxDepthSlider.Value < MinDepthSlider.Value + 0.2)
            {
                if (ReferenceEquals(sender, MinDepthSlider))
                {
                    MaxDepthSlider.Value = MinDepthSlider.Value + 0.2;
                }
                else
                {
                    MinDepthSlider.Value = MaxDepthSlider.Value - 0.2;
                }
            }

            MinDepthLabel.Text = MinDepthSlider.Value.ToString("0.00") + " m";
            MaxDepthLabel.Text = MaxDepthSlider.Value.ToString("0.00") + " m";

            VolumePreset preset = CurrentPreset;
            if (preset != null)
            {
                DepthRangeHint.Text = string.Format(
                    "Scanned region: {0:0.00}–{1:0.00} m from the camera (volume is {2:0.0} m deep, starting at the near clip). Keep it tight around your body.",
                    MinDepthSlider.Value,
                    Math.Min(MaxDepthSlider.Value, MinDepthSlider.Value + preset.SizeZ),
                    preset.SizeZ);
            }
        }

        private void WeightSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!initialized)
            {
                return;
            }

            WeightLabel.Text = ((int)WeightSlider.Value).ToString();
        }

        private void CaptureColorCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (!initialized)
            {
                return;
            }

            ColorViewCheck.IsEnabled = CaptureColorCheck.IsChecked == true;
            if (CaptureColorCheck.IsChecked != true)
            {
                ColorViewCheck.IsChecked = false;
            }

            // Re-subscribe so the color stream is only transferred when needed
            if (sensor != null)
            {
                OpenReader();
                StatusText.Text = CaptureColorCheck.IsChecked == true
                    ? "Color stream on."
                    : "Color stream off — USB bandwidth roughly halved.";
            }
        }

        private void AutoRecoveryCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (!initialized)
            {
                return;
            }

            lock (engineLock)
            {
                engine.AutoRecoveryEnabled = AutoRecoveryCheck.IsChecked == true;
            }
        }

        private void SmoothDepthCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (!initialized)
            {
                return;
            }

            lock (engineLock)
            {
                engine.SmoothDepth = SmoothDepthCheck.IsChecked == true;
            }
        }

        private void UpdateUiState()
        {
            bool idle = state == ScanState.Idle;
            StartButton.IsEnabled = volumeReady;
            StartButton.Content = idle ? "▶  Start Scan" : "▶  Restart Scan";
            PauseButton.IsEnabled = !idle;
            PauseButton.Content = state == ScanState.Paused ? "⏵  Resume" : "⏸  Pause";
            ExportButton.IsEnabled = !idle || (volumeReady && engine.FramesIntegrated > 0);
            PresetCombo.IsEnabled = idle;
            CaptureColorCheck.IsEnabled = idle;
            IdleHint.Visibility = idle ? Visibility.Visible : Visibility.Collapsed;

            if (idle)
            {
                MainImage.Source = depthPreviewBitmap;
                TrackingDot.Fill = Brushes.Gray;
                TrackingText.Text = "Idle";
            }
        }

        // ------------------------------------------------------------------
        // Audio cues
        // ------------------------------------------------------------------

        /// <summary>
        /// Called on the UI thread for each processed frame. Plays one-shot tones on
        /// good↔lost transitions; the repeating heartbeat/alert is driven by the timer.
        /// </summary>
        private void UpdateAudioState(bool trackingOk, int consecutiveFailures, bool sparseView)
        {
            if (soundCues == null || AudioCuesCheck.IsChecked != true || state != ScanState.Scanning)
            {
                return;
            }

            AudioCue next;
            if (trackingOk)
            {
                next = AudioCue.Good;
            }
            else if (sparseView)
            {
                next = AudioCue.Quiet; // a depth-coverage problem, not the user going too fast
            }
            else if (consecutiveFailures >= AudioLostThreshold)
            {
                next = AudioCue.Lost;
            }
            else
            {
                next = audioCue; // hysteresis: hold through a stray failure
            }

            if (next != audioCue)
            {
                if (next == AudioCue.Lost)
                {
                    soundCues.PlayLost();                 // → genuinely lost
                    lastAlert = DateTime.UtcNow;
                }
                else if (next == AudioCue.Good && audioCue == AudioCue.Lost)
                {
                    soundCues.PlayRecovered();            // lost → regained
                    lastHeartbeat = DateTime.UtcNow;
                }
                else if (next == AudioCue.Good)
                {
                    lastHeartbeat = DateTime.UtcNow;      // resume heartbeat fresh after a quiet spell
                }

                audioCue = next;
            }
        }

        private void AudioTimer_Tick(object sender, EventArgs e)
        {
            if (soundCues == null || AudioCuesCheck.IsChecked != true || state != ScanState.Scanning)
            {
                return;
            }

            DateTime now = DateTime.UtcNow;
            if (audioCue == AudioCue.Lost)
            {
                if ((now - lastAlert).TotalMilliseconds >= AlertIntervalMs)
                {
                    soundCues.PlayLost();
                    lastAlert = now;
                }
            }
            else if (audioCue == AudioCue.Good)
            {
                if ((now - lastHeartbeat).TotalMilliseconds >= HeartbeatIntervalMs)
                {
                    soundCues.PlayHeartbeat();
                    lastHeartbeat = now;
                }
            }
            // AudioCue.Quiet: say nothing
        }

        private void AudioCuesCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (!initialized || soundCues == null)
            {
                return;
            }

            // Preview the heartbeat tick when the user turns cues on.
            if (AudioCuesCheck.IsChecked == true)
            {
                soundCues.PlayHeartbeat();
            }
        }

        private void StatusTimer_Tick(object sender, EventArgs e)
        {
            int depthFps = Interlocked.Exchange(ref depthFpsCounter, 0);
            int fusedFps = Interlocked.Exchange(ref fusedFpsCounter, 0);

            if (state == ScanState.Scanning)
            {
                FpsText.Text = string.Format("Depth {0} fps · Fused {1} fps · Fusion {2} ms",
                    depthFps, fusedFps, lastFusionMs);
            }
            else
            {
                FpsText.Text = string.Format("Depth {0} fps", depthFps);
            }
        }
    }
}

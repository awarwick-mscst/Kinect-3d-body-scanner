# Kinect 3D Body Scanner

Scan yourself into a true 360° 3D model with a Kinect for Windows v2 (or Kinect for
Xbox One + Windows adapter).

Built on **Kinect Fusion** (ships with the Kinect for Windows SDK 2.0): every depth
frame is tracked against the model built so far and fused into a single volumetric
reconstruction, so the "multiple scans" are stitched together continuously and
automatically as you rotate. Optionally captures the color camera too, giving you a
textured (vertex-colored) model.

## Requirements

- Windows 10/11, Kinect v2 sensor on a **USB 3.0** port
- Kinect for Windows SDK 2.0 installed (default path `C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409`)
- A DirectX 11 capable GPU (the app falls back to CPU, but it is far too slow for live scanning)
- .NET Framework 4.8 (preinstalled on Windows 10/11)

## Build & run

```powershell
dotnet build -c Release
.\bin\Release\net48\KinectScanner.exe
```

## How to scan yourself (swivel chair method)

1. **Camera setup:** put the Kinect on a tripod or shelf at chest height, level,
   pointing at open space (no walls/furniture within ~3 m behind you if possible).
2. **Pick a preset** — *Bust* (highest detail), *Seated person*, or *Full body, standing*.
   Each preset shows a tip with the rotation speed it needs. Note that **Bust is the
   most motion-sensitive** mode: at close range a small turn moves a lot across the
   sensor, and a head has little geometry to track, so it needs a *very* slow turn
   (~2 min) and your shoulders kept in frame. If you're finding it fiddly, *Seated
   person* is far more forgiving and still captures good facial detail.
3. **Sit/stand at the right distance** and use the live depth preview to frame
   yourself: your body should be bright, everything else dark.
4. **Tighten the depth window:** set the far clip just behind your back and the near
   clip just in front of you. This is the key step — Kinect Fusion thinks the world
   is rigid, so the static room must be clipped out of view; then your slow rotation
   looks (to the tracker) like the camera orbiting around you.
5. Press **Start Scan**. Hold perfectly still for ~2 seconds while the first frames fuse.
6. **Rotate slowly and smoothly** — aim for one revolution in ~45–90 seconds,
   keeping your pose frozen (arms glued to your body or armrests, head still
   relative to shoulders). On a swivel chair push yourself around with your toes;
   standing, shuffle in small steps on the spot. Smoothness matters more than
   absolute speed — a steady glide tracks better than a slow-but-jerky turn.
7. Watch the model build in the main view. With **Auto-recover lost tracking** on
   (default), if you move too fast the status shows **Recovering…** — just stop and
   hold still for a second and it relocalizes itself (dot flashes blue =
   "Recovered"), then carry on. If it can't recover, the red banner asks you to
   rotate back slowly toward your last good position.
8. After a bit more than a full turn (overlap helps close seams), press **Pause**,
   then **Export Mesh…**.

Alternative mode: you stand perfectly still and a helper slowly walks/orbits the
Kinect around you (handheld, smooth movements). Same settings apply.

## Export formats

| Format | Color | Notes |
|--------|-------|-------|
| `.ply` (binary) | vertex colors | Best choice — open in [MeshLab](https://www.meshlab.net/) or Blender |
| `.obj` (ASCII) | vertex colors (non-standard extension, read by Blender/MeshLab) | Large files |
| `.stl` (binary) | none | For 3D printing |

**Units are meters.** Many 3D-print slicers assume millimeters, so scale by ×1000
when importing the STL. The mesh is raw scanner output — for printing, run it
through MeshLab/Blender for hole filling (e.g. Poisson reconstruction) and cleanup.

## Settings reference

- **Scan volume preset** — physical size and detail of the reconstruction box.
  Detail is voxels-per-meter (384/m ≈ 2.6 mm voxels for the bust preset).
- **Depth window** — only geometry between the near and far clip is fed to the
  scanner. The reconstruction volume itself starts at the near clip and extends
  the preset's depth (so for the full-body preset, 1.4 m near clip → volume covers
  1.4–2.9 m).
- **Integration weight** — how many frames are averaged per voxel. Lower values
  adapt faster to slight body movement (good for people), higher values give
  smoother, more noise-free surfaces (good for rigid objects).
- **Capture color** — fuses the 1080p color camera into the model (vertex colors).
  Costs some performance and USB bandwidth; uncheck it if your frame rate is low or
  the sensor connection is marginal.
- **Auto-recover lost tracking** — continuously stores keyframe poses while scanning
  (via the SDK's `CameraPoseFinder`) and, when tracking fails for a few frames,
  searches that database for the current viewpoint and snaps the camera back onto it.
  This is what lets you rotate at a natural speed instead of nailing a perfect slow
  turn. Recovery uses the depth silhouette (not RGB), so it works with color off too.
- **Smooth depth (steadier tracking)** — bilateral-filters the depth before tracking
  (SDK `SmoothDepthFloatFrame`) so the ICP tracker has a cleaner surface to lock onto.
  This is the main fix for losing tracking on hard surfaces — mesh or dark office-chair
  backs, and flat featureless panels. On by default; turn it off only if you want the
  absolute maximum fine detail on a well-lit, feature-rich subject.
- **Audio cues (scan by ear)** — maps the tracking state to sound so you don't have to
  watch the screen while you turn: a soft tick every ~2 s means tracking is good, a
  repeating descending buzz means tracking is lost (slow down / hold still), and a
  rising chime means it recovered. Tones are synthesized in memory (no sound files)
  and play through your default audio device. When the view is too feature-poor to
  track fairly (e.g. a dark mesh chair back fills the frame), the buzz is suppressed
  and the status shows "Sparse view" instead of falsely nagging you to slow down.

## High-detail faces: Color Burst → photogrammetry

The Kinect's depth sensor is too coarse for crisp facial detail (it resolves only
~2–3 mm and a face moves slightly during the slow scan, so features "melt"). For a
sharp face, use **photogrammetry** instead — reconstructing geometry from ordinary
high-resolution photos, which carry far more detail than depth.

The **Color Burst** button captures the Kinect's full-resolution (1920×1080) color
camera as a sequence of JPEGs while you slowly turn:

1. Press **📷 Start Color Burst**. A timestamped folder is created under
   `Documents\KinectScans\colorburst_<date>`.
2. Turn slowly through the angles you want (one revolution, or just the face). Photos
   are saved ~3×/second; the status shows the running count.
3. Press **⏹ Stop Color Burst** — the folder opens automatically.
4. Process the folder in Meshroom (steps below) to build a textured mesh.

Tips for good photogrammetry input: **bright, even lighting** (the color camera needs
light and slows its shutter in the dark, causing blur); **rotate slowly** to avoid
motion blur; keep a **neutral, frozen expression**; ensure lots of overlap between
consecutive shots (the slow turn handles this). A phone camera will give even better
results than the Kinect's color camera if you want maximum quality — the same software
workflow applies.

### Running Meshroom (manual workflow)

1. **Install:** download Meshroom from the
   [AliceVision GitHub releases](https://github.com/alicevision/Meshroom/releases)
   (Windows `.zip`) or [FOSSHub](https://www.fosshub.com/Meshroom.html). It is portable —
   unzip anywhere and run `Meshroom.exe`. It needs an NVIDIA CUDA GPU for the dense step.
2. **⚠️ Background vs. rotation — the #1 cause of failed self-scans.** Photogrammetry
   solves camera positions from matched features. If *you* rotate while the camera and
   background stay still, Meshroom sees a static background plus a spinning subject and
   can't solve it (the same rigidity assumption the Kinect tracker has). Fix it one of
   two ways: have a helper move the camera around your *stationary* head (best), or hang
   a **plain, featureless backdrop** (a bedsheet) behind you so only your face drives the
   solve. A cluttered room behind a rotating you will fail.
3. **Cull blurry photos** before importing — blur corrupts the reconstruction.
4. In Meshroom: **File → Save As** (so output lands next to the project), **drag your
   photo folder** into the *Images* panel, then click the green **Start** button.
5. Wait for the pipeline (~10 nodes). ~150 photos on a 2080 Ti ≈ 20–60 min.
6. Output: `<project>\MeshroomCache\Texturing\<id>\texturedMesh.obj` (+ `.mtl` + texture).
7. **Combine (optional):** in Blender/MeshLab, use the Kinect mesh for the body and the
   Meshroom mesh for the face, align, and stitch — clean body + sharp textured face.

## Preparing the model for 3D printing

The raw scan includes the floor plane, the chair, and a noisy base — fine for viewing,
but you'll want to clean it before slicing in Cura:

1. Open the `.ply`/`.stl` in **Blender** or **MeshLab**.
2. Delete the floor/chair: select those faces and remove them (in Blender, edit mode →
   box-select → delete; in MeshLab, *Select Faces in a rectangular region* → delete).
3. Fill holes and make it watertight: MeshLab *Filters → Remeshing → Screened Poisson
   Surface Reconstruction*, or Blender's *Remesh* modifier. A printable model must be
   a closed (manifold) surface.
4. Stand it up / flatten the base so it has a footprint to print on.
5. Export as STL. Remember the scale: the scan is in **meters**, so a ~1.7 m tall scan
   imports as 1.7 mm in a millimeter-based slicer — **scale ×1000**, then resize to the
   height you actually want to print.

## Troubleshooting

- **"Access is denied" when launching the exe** — Microsoft Defender's Attack
  Surface Reduction rule *"Block executable files from running unless they meet a
  prevalence, age, or trusted list criterion"* blocks freshly compiled local
  binaries (Defender event ID 1121, rule `01443614-CD74-433A-B99E-2ECDC07BFC25`).
  Fix: add an **ASR-only exclusion** for this project's `bin` folder — this scopes
  only the ASR rules and does not weaken normal antivirus scanning.
  - On an unmanaged PC (admin PowerShell):
    `Add-MpPreference -AttackSurfaceReductionOnlyExclusions "<project>\bin"`
  - On an Intune/Defender-managed PC (tamper protection from ATP), local changes
    won't stick — add the exclusion in the Intune admin center instead:
    *Endpoint security → Attack surface reduction → (your ASR policy) →*
    **Attack Surface Reduction Only Exclusions** (or the per-rule exclusion list
    under the rule itself), then sync the device from Settings → Accounts →
    Access work or school.
- **"Kinect not detected"** — must be a USB 3.0 port (ideally Intel/AMD chipset, not
  a hub) and the Kinect's power brick plugged in. Test with *SDK Browser v2.0 →
  Kinect Configuration Verifier*.
- **Kinect connects then drops every ~15 seconds, in a loop** — a perfectly periodic
  connect/disconnect cycle (visible as a rising dropout counter in the status bar and
  in `scanner.log`) is almost always a *software* reset loop, not bad hardware. The
  usual culprit on Windows 11 is **audio enhancements on the Kinect's microphone**:
  *Settings → System → Sound → Input → "Microphone Array (Xbox NUI Sensor)" → Audio
  enhancements → Off*, then unplug/replug the sensor. Also make sure that mic isn't
  muted (a muted Kinect mic makes the runtime restart the device repeatedly). Disabling
  **USB selective suspend** in your power plan helps on marginal links too. These fixes
  are independent of the USB controller — the loop persists across ports/reboots until
  the audio issue is addressed.
- **Tracking lost immediately / constantly** — depth window too tight or too much
  static background in view; you're rotating too fast; or you're closer than 0.5 m
  (hardware minimum). Make sure *you* fill most of the in-range (bright) pixels.
- **Ghosting / doubled limbs** — you moved relative to yourself (lifted an arm,
  tilted your head). Reset and rescan, holding the pose rigid.
- **Tracking freaks out on a chair back / blank wall / flat panel** — featureless or
  IR-absorbing surfaces (dark mesh chair backs especially) starve the geometric
  tracker, so it stalls even when you're barely moving. Mitigations, in order:
  keep **Smooth depth** on; make sure *you* (head/shoulders/arms — feature-rich
  geometry) stay in frame rather than the camera ending up pointed at a bare panel;
  drape a textured cloth or tape a few markers on a totally flat back to give the
  tracker something to grip. The status shows **Sparse view** (and stays quiet)
  instead of buzzing when this is the surface's fault rather than yours.
- **Low fps** — check the status bar: if it says CPU mode, your GPU lacks DX11
  support. Also close other GPU-heavy apps; disable color capture.
- **Volume allocation fails** — not enough GPU memory for the preset; pick a smaller
  preset or close other applications.

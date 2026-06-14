using System;
using System.IO;
using System.Media;

namespace KinectScanner
{
    /// <summary>
    /// Generates short tones in memory and plays them as non-blocking audio cues, so
    /// the user can scan by ear without watching the screen. No external sound files:
    /// each cue is a little PCM WAV synthesized at construction and played via
    /// <see cref="SoundPlayer"/> (which plays asynchronously on its own thread).
    /// </summary>
    public sealed class SoundCues : IDisposable
    {
        private const int SampleRate = 44100;

        private readonly SoundPlayer heartbeat;
        private readonly SoundPlayer lost;
        private readonly SoundPlayer recovered;
        private readonly SoundPlayer milestone;
        private readonly SoundPlayer targetReached;

        public SoundCues()
        {
            // Good: one soft, high, short tick.
            heartbeat = MakePlayer(new[] { new Tone(1046.5, 55, 0.16) });
            // Lost: louder descending two-note buzz — clearly "something's wrong".
            lost = MakePlayer(new[] { new Tone(440.0, 140, 0.5), new Tone(311.1, 180, 0.5) });
            // Regained: pleasant rising two-note chime.
            recovered = MakePlayer(new[] { new Tone(659.25, 90, 0.42), new Tone(987.77, 140, 0.42) });
            // Quarter-turn tick: a single mid "bong", distinct from the high heartbeat.
            milestone = MakePlayer(new[] { new Tone(587.33, 90, 0.34) });
            // Target reached (full turn): celebratory rising three-note arpeggio.
            targetReached = MakePlayer(new[]
            {
                new Tone(659.25, 110, 0.45), new Tone(830.61, 110, 0.45), new Tone(1046.5, 170, 0.45),
            });
        }

        public void PlayHeartbeat() { SafePlay(heartbeat); }
        public void PlayLost() { SafePlay(lost); }
        public void PlayRecovered() { SafePlay(recovered); }
        public void PlayMilestone() { SafePlay(milestone); }
        public void PlayTargetReached() { SafePlay(targetReached); }

        private static void SafePlay(SoundPlayer player)
        {
            try { player.Play(); }
            catch (Exception) { /* audio must never break the scan */ }
        }

        private struct Tone
        {
            public readonly double Freq;
            public readonly int Ms;
            public readonly double Gain;
            public Tone(double freq, int ms, double gain) { Freq = freq; Ms = ms; Gain = gain; }
        }

        private static SoundPlayer MakePlayer(Tone[] segments)
        {
            var stream = new MemoryStream();
            WriteWav(stream, segments);
            stream.Position = 0;
            var player = new SoundPlayer(stream);
            try { player.Load(); }
            catch (Exception) { }
            return player;
        }

        private static void WriteWav(Stream output, Tone[] segments)
        {
            int totalSamples = 0;
            foreach (Tone s in segments)
            {
                totalSamples += SampleRate * s.Ms / 1000;
            }

            var samples = new short[totalSamples];
            int idx = 0;
            int ramp = SampleRate * 5 / 1000; // 5 ms attack/decay to avoid clicks
            foreach (Tone seg in segments)
            {
                int n = SampleRate * seg.Ms / 1000;
                for (int i = 0; i < n; i++)
                {
                    double env = 1.0;
                    if (i < ramp) env = (double)i / ramp;
                    else if (i > n - ramp) env = (double)(n - i) / ramp;
                    double sample = Math.Sin(2.0 * Math.PI * seg.Freq * i / SampleRate) * seg.Gain * env;
                    samples[idx++] = (short)(sample * short.MaxValue);
                }
            }

            int dataBytes = samples.Length * 2;
            var w = new BinaryWriter(output);
            WriteAscii(w, "RIFF");
            w.Write(36 + dataBytes);
            WriteAscii(w, "WAVE");
            WriteAscii(w, "fmt ");
            w.Write(16);              // PCM fmt chunk size
            w.Write((short)1);        // PCM
            w.Write((short)1);        // mono
            w.Write(SampleRate);
            w.Write(SampleRate * 2);  // byte rate (mono, 16-bit)
            w.Write((short)2);        // block align
            w.Write((short)16);       // bits per sample
            WriteAscii(w, "data");
            w.Write(dataBytes);
            foreach (short s in samples)
            {
                w.Write(s);
            }
            w.Flush();
        }

        private static void WriteAscii(BinaryWriter w, string s)
        {
            foreach (char c in s)
            {
                w.Write((byte)c);
            }
        }

        public void Dispose()
        {
            if (heartbeat != null) { heartbeat.Dispose(); }
            if (lost != null) { lost.Dispose(); }
            if (recovered != null) { recovered.Dispose(); }
            if (milestone != null) { milestone.Dispose(); }
            if (targetReached != null) { targetReached.Dispose(); }
        }
    }
}

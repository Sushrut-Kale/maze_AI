#pragma once
// ============================================================
// Recorder.h  —  MAZEAI Toggle-Gated Frame Capture
//   Reads raw framebuffer pixels and saves as PPM P6 binary.
//   Recording ONLY happens when isRecording == true.
//   Toggle with F9 key.
// ============================================================
#include <GL/glut.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

// Platform-specific directory creation
#if defined(_WIN32)
  #include <direct.h>
  #define MAZEAI_MKDIR(d) _mkdir(d)
#else
  #include <sys/stat.h>
  #define MAZEAI_MKDIR(d) mkdir(d, 0777)
#endif

class FrameRecorder {
public:
    bool        isRecording = false;
    int         frameCount  = 0;
    std::string folder;

    explicit FrameRecorder(const std::string& folderName = "frames")
        : folder(folderName) {}

    void toggle() {
        isRecording = !isRecording;
        if (isRecording) {
            ensureFolder();
            std::cout << "[Recorder] Started  — saving to " << folder << "/\n";
        } else {
            std::cout << "[Recorder] Stopped  — " << frameCount << " frames saved.\n";
        }
    }

    // Call once per timer tick; skips silently when not recording
    void captureFrame(int width, int height) {
        if (!isRecording) return;

        std::vector<unsigned char> pixels((size_t)width * height * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        std::ostringstream ss;
        ss << folder << "/frame"
           << std::setw(5) << std::setfill('0') << frameCount++ << ".ppm";

        std::ofstream f(ss.str(), std::ios::binary);
        if (!f) {
            std::cerr << "[Recorder] Cannot write: " << ss.str() << '\n';
            return;
        }

        // PPM P6 header
        f << "P6\n" << width << ' ' << height << "\n255\n";

        // Flip rows (OpenGL origin is bottom-left)
        for (int row = height - 1; row >= 0; row--)
            f.write(reinterpret_cast<char*>(&pixels[(size_t)row * width * 3]),
                    (std::streamsize)(width * 3));
    }

private:
    bool folderMade = false;
    void ensureFolder() {
        if (!folderMade) {
            MAZEAI_MKDIR(folder.c_str());
            folderMade = true;
        }
    }
};

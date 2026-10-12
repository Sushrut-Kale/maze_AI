#pragma once
// ============================================================
// GL/glut.h  —  Native Windows GLUT Implementation for MazeAI
//   Zero external DLL dependency: uses Windows GDI + OpenGL32
//   Compatible with 32-bit and 64-bit MinGW / GCC / MSVC
// ============================================================

#ifndef __NATIVE_GLUT_H__
#define __NATIVE_GLUT_H__

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <vector>
#include <map>
#include <string>
#include <cstring>
#include <cmath>
#include <chrono>

// ── Display Mode Constants ──────────────────────────────────
#define GLUT_RGB                 0x0000
#define GLUT_RGBA                0x0000
#define GLUT_INDEX               0x0001
#define GLUT_SINGLE              0x0000
#define GLUT_DOUBLE              0x0002
#define GLUT_ACCUM               0x0004
#define GLUT_ALPHA               0x0008
#define GLUT_DEPTH               0x0010
#define GLUT_STENCIL             0x0020

// ── Mouse Constants ─────────────────────────────────────────
#define GLUT_LEFT_BUTTON         0
#define GLUT_MIDDLE_BUTTON       1
#define GLUT_RIGHT_BUTTON        2
#define GLUT_DOWN                0
#define GLUT_UP                  1

// ── Special Key Constants ───────────────────────────────────
#define GLUT_KEY_F1              1
#define GLUT_KEY_F2              2
#define GLUT_KEY_F3              3
#define GLUT_KEY_F4              4
#define GLUT_KEY_F5              5
#define GLUT_KEY_F6              6
#define GLUT_KEY_F7              7
#define GLUT_KEY_F8              8
#define GLUT_KEY_F9              9
#define GLUT_KEY_F10             10
#define GLUT_KEY_F11             11
#define GLUT_KEY_F12             12
#define GLUT_KEY_LEFT            100
#define GLUT_KEY_UP              101
#define GLUT_KEY_RIGHT           102
#define GLUT_KEY_DOWN            103
#define GLUT_KEY_PAGE_UP         104
#define GLUT_KEY_PAGE_DOWN       105
#define GLUT_KEY_HOME            106
#define GLUT_KEY_END             107
#define GLUT_KEY_INSERT          108

// ── Font Handles ────────────────────────────────────────────
#define GLUT_BITMAP_HELVETICA_10       ((void*)1)
#define GLUT_BITMAP_HELVETICA_12       ((void*)2)
#define GLUT_BITMAP_HELVETICA_18       ((void*)3)
#define GLUT_BITMAP_TIMES_ROMAN_10     ((void*)4)
#define GLUT_BITMAP_TIMES_ROMAN_24     ((void*)5)
#define GLUT_BITMAP_8_BY_13            ((void*)6)
#define GLUT_BITMAP_9_BY_15            ((void*)7)
#define GLUT_STROKE_ROMAN              ((void*)8)
#define GLUT_STROKE_MONO_ROMAN         ((void*)9)

// ── Internal State Structure ────────────────────────────────
namespace NativeGLUT {
    struct TimerEntry {
        unsigned int id;
        std::chrono::steady_clock::time_point triggerTime;
        void (*callback)(int);
        int value;
    };

    struct Context {
        HWND hWnd = NULL;
        HDC hDC = NULL;
        HGLRC hRC = NULL;
        int winWidth = 800;
        int winHeight = 600;
        std::string winTitle = "OpenGL Application";
        unsigned int displayMode = GLUT_DOUBLE | GLUT_RGB;

        void (*displayFunc)(void) = nullptr;
        void (*reshapeFunc)(int, int) = nullptr;
        void (*keyboardFunc)(unsigned char, int, int) = nullptr;
        void (*specialFunc)(int, int, int) = nullptr;
        void (*mouseFunc)(int, int, int, int) = nullptr;
        void (*motionFunc)(int, int) = nullptr;
        void (*passiveMotionFunc)(int, int) = nullptr;

        int mouseX = 0;
        int mouseY = 0;

        std::vector<TimerEntry> timers;
        unsigned int nextTimerId = 1;

        // Fonts
        GLuint fontBaseHelvetica10 = 0;
        GLuint fontBaseHelvetica12 = 0;
        GLuint fontBaseHelvetica18 = 0;
        GLuint fontBaseTimes10 = 0;
        GLuint fontBaseTimes24 = 0;
        GLuint fontBaseFixed8x13 = 0;
        GLuint fontBaseFixed9x15 = 0;
        GLuint fontBaseStroke = 0;
        GLYPHMETRICSFLOAT strokeMetrics[256];
        bool fontsInitialized = false;
    };

    inline Context& getContext() {
        static Context ctx;
        return ctx;
    }

    inline void initFonts(HDC hDC) {
        Context& ctx = getContext();
        if (ctx.fontsInitialized) return;

        auto createBitmapFont = [&](const char* face, int size, int weight) -> GLuint {
            HFONT hFont = CreateFontA(
                -size, 0, 0, 0, weight, FALSE, FALSE, FALSE,
                ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
                ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, face
            );
            HFONT hOldFont = (HFONT)SelectObject(hDC, hFont);
            GLuint base = glGenLists(256);
            wglUseFontBitmapsA(hDC, 0, 256, base);
            SelectObject(hDC, hOldFont);
            DeleteObject(hFont);
            return base;
        };

        ctx.fontBaseHelvetica10 = createBitmapFont("Arial", 11, FW_NORMAL);
        ctx.fontBaseHelvetica12 = createBitmapFont("Arial", 13, FW_NORMAL);
        ctx.fontBaseHelvetica18 = createBitmapFont("Arial", 18, FW_BOLD);
        ctx.fontBaseTimes10     = createBitmapFont("Times New Roman", 11, FW_NORMAL);
        ctx.fontBaseTimes24     = createBitmapFont("Times New Roman", 24, FW_BOLD);
        ctx.fontBaseFixed8x13   = createBitmapFont("Courier New", 13, FW_NORMAL);
        ctx.fontBaseFixed9x15   = createBitmapFont("Courier New", 15, FW_BOLD);

        // Stroke font via wglUseFontOutlines
        HFONT hStrokeFont = CreateFontA(
            -119, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
            ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
            ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, "Arial"
        );
        HFONT hOld = (HFONT)SelectObject(hDC, hStrokeFont);
        ctx.fontBaseStroke = glGenLists(256);
        std::memset(ctx.strokeMetrics, 0, sizeof(ctx.strokeMetrics));
        wglUseFontOutlinesA(hDC, 0, 256, ctx.fontBaseStroke, 0.0f, 0.0f, WGL_FONT_LINES, ctx.strokeMetrics);
        SelectObject(hDC, hOld);
        DeleteObject(hStrokeFont);

        ctx.fontsInitialized = true;
    }

    inline LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
        Context& ctx = getContext();
        switch (uMsg) {
            case WM_PAINT: {
                PAINTSTRUCT ps;
                BeginPaint(hWnd, &ps);
                if (ctx.displayFunc) ctx.displayFunc();
                EndPaint(hWnd, &ps);
                return 0;
            }
            case WM_SIZE: {
                int w = LOWORD(lParam);
                int h = HIWORD(lParam);
                ctx.winWidth = w;
                ctx.winHeight = h;
                if (ctx.reshapeFunc) ctx.reshapeFunc(w, h);
                return 0;
            }
            case WM_MOUSEMOVE: {
                int x = LOWORD(lParam);
                int y = HIWORD(lParam);
                ctx.mouseX = x;
                ctx.mouseY = y;
                if (wParam & (MK_LBUTTON | MK_MBUTTON | MK_RBUTTON)) {
                    if (ctx.motionFunc) ctx.motionFunc(x, y);
                } else {
                    if (ctx.passiveMotionFunc) ctx.passiveMotionFunc(x, y);
                }
                return 0;
            }
            case WM_LBUTTONDOWN:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_LEFT_BUTTON, GLUT_DOWN, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_LBUTTONUP:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_LEFT_BUTTON, GLUT_UP, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_RBUTTONDOWN:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_RIGHT_BUTTON, GLUT_DOWN, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_RBUTTONUP:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_RIGHT_BUTTON, GLUT_UP, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_MBUTTONDOWN:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_MIDDLE_BUTTON, GLUT_DOWN, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_MBUTTONUP:
                if (ctx.mouseFunc) ctx.mouseFunc(GLUT_MIDDLE_BUTTON, GLUT_UP, LOWORD(lParam), HIWORD(lParam));
                return 0;
            case WM_CHAR:
                if (ctx.keyboardFunc) ctx.keyboardFunc((unsigned char)wParam, ctx.mouseX, ctx.mouseY);
                return 0;
            case WM_KEYDOWN: {
                int key = 0;
                switch (wParam) {
                    case VK_LEFT:   key = GLUT_KEY_LEFT; break;
                    case VK_RIGHT:  key = GLUT_KEY_RIGHT; break;
                    case VK_UP:     key = GLUT_KEY_UP; break;
                    case VK_DOWN:   key = GLUT_KEY_DOWN; break;
                    case VK_PRIOR:  key = GLUT_KEY_PAGE_UP; break;
                    case VK_NEXT:   key = GLUT_KEY_PAGE_DOWN; break;
                    case VK_HOME:   key = GLUT_KEY_HOME; break;
                    case VK_END:    key = GLUT_KEY_END; break;
                    case VK_INSERT: key = GLUT_KEY_INSERT; break;
                    case VK_F1:     key = GLUT_KEY_F1; break;
                    case VK_F2:     key = GLUT_KEY_F2; break;
                    case VK_F3:     key = GLUT_KEY_F3; break;
                    case VK_F4:     key = GLUT_KEY_F4; break;
                    case VK_F5:     key = GLUT_KEY_F5; break;
                    case VK_F6:     key = GLUT_KEY_F6; break;
                    case VK_F7:     key = GLUT_KEY_F7; break;
                    case VK_F8:     key = GLUT_KEY_F8; break;
                    case VK_F9:     key = GLUT_KEY_F9; break;
                    case VK_F10:    key = GLUT_KEY_F10; break;
                    case VK_F11:    key = GLUT_KEY_F11; break;
                    case VK_F12:    key = GLUT_KEY_F12; break;
                }
                if (key != 0 && ctx.specialFunc) {
                    ctx.specialFunc(key, ctx.mouseX, ctx.mouseY);
                }
                return 0;
            }
            case WM_DESTROY:
                PostQuitMessage(0);
                return 0;
        }
        return DefWindowProcA(hWnd, uMsg, wParam, lParam);
    }
} // namespace NativeGLUT

// ── GLUT API Implementations ─────────────────────────────────

inline void glutInit(int* argc, char** argv) {
    (void)argc; (void)argv;
}

inline void glutInitDisplayMode(unsigned int mode) {
    NativeGLUT::getContext().displayMode = mode;
}

inline void glutInitWindowSize(int width, int height) {
    NativeGLUT::getContext().winWidth = width;
    NativeGLUT::getContext().winHeight = height;
}

inline int glutCreateWindow(const char* title) {
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    ctx.winTitle = title ? title : "OpenGL Window";

    HINSTANCE hInstance = GetModuleHandle(NULL);
    WNDCLASSEXA wc;
    std::memset(&wc, 0, sizeof(wc));
    wc.cbSize = sizeof(WNDCLASSEXA);
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wc.lpfnWndProc = NativeGLUT::WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.lpszClassName = "MazeAIOpenGLWindowClass";
    RegisterClassExA(&wc);

    RECT rect = {0, 0, ctx.winWidth, ctx.winHeight};
    AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, WS_EX_APPWINDOW);
    int w = rect.right - rect.left;
    int h = rect.bottom - rect.top;

    ctx.hWnd = CreateWindowExA(
        WS_EX_APPWINDOW,
        "MazeAIOpenGLWindowClass",
        ctx.winTitle.c_str(),
        WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
        CW_USEDEFAULT, CW_USEDEFAULT,
        w, h,
        NULL, NULL, hInstance, NULL
    );

    ctx.hDC = GetDC(ctx.hWnd);

    PIXELFORMATDESCRIPTOR pfd;
    std::memset(&pfd, 0, sizeof(pfd));
    pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int format = ChoosePixelFormat(ctx.hDC, &pfd);
    SetPixelFormat(ctx.hDC, format, &pfd);

    ctx.hRC = wglCreateContext(ctx.hDC);
    wglMakeCurrent(ctx.hDC, ctx.hRC);

    ShowWindow(ctx.hWnd, SW_SHOW);
    UpdateWindow(ctx.hWnd);

    NativeGLUT::initFonts(ctx.hDC);
    return 1;
}

inline void glutDisplayFunc(void (*func)(void)) {
    NativeGLUT::getContext().displayFunc = func;
}

inline void glutReshapeFunc(void (*func)(int, int)) {
    NativeGLUT::getContext().reshapeFunc = func;
}

inline void glutKeyboardFunc(void (*func)(unsigned char, int, int)) {
    NativeGLUT::getContext().keyboardFunc = func;
}

inline void glutSpecialFunc(void (*func)(int, int, int)) {
    NativeGLUT::getContext().specialFunc = func;
}

inline void glutMouseFunc(void (*func)(int, int, int, int)) {
    NativeGLUT::getContext().mouseFunc = func;
}

inline void glutMotionFunc(void (*func)(int, int)) {
    NativeGLUT::getContext().motionFunc = func;
}

inline void glutPassiveMotionFunc(void (*func)(int, int)) {
    NativeGLUT::getContext().passiveMotionFunc = func;
}

inline void glutPostRedisplay(void) {
    HWND hWnd = NativeGLUT::getContext().hWnd;
    if (hWnd) {
        InvalidateRect(hWnd, NULL, FALSE);
    }
}

inline void glutSwapBuffers(void) {
    HDC hDC = NativeGLUT::getContext().hDC;
    if (hDC) {
        SwapBuffers(hDC);
    }
}

inline void glutTimerFunc(unsigned int msecs, void (*func)(int), int value) {
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    NativeGLUT::TimerEntry te;
    te.id = ctx.nextTimerId++;
    te.triggerTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(msecs);
    te.callback = func;
    te.value = value;
    ctx.timers.push_back(te);
}

inline void glutMainLoop(void) {
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    MSG msg;

    while (true) {
        while (PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) return;
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }

        // Process timers
        auto now = std::chrono::steady_clock::now();
        std::vector<NativeGLUT::TimerEntry> dueTimers;
        std::vector<NativeGLUT::TimerEntry> remainingTimers;

        for (auto& t : ctx.timers) {
            if (t.triggerTime <= now) dueTimers.push_back(t);
            else remainingTimers.push_back(t);
        }
        ctx.timers = remainingTimers;

        for (auto& t : dueTimers) {
            if (t.callback) t.callback(t.value);
        }

        Sleep(1);
    }
}

// ── Font Drawing Functions ───────────────────────────────────

inline void glutBitmapCharacter(void* font, int character) {
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    if (character < 0 || character > 255) return;

    GLuint base = ctx.fontBaseHelvetica12;
    if (font == GLUT_BITMAP_HELVETICA_10)      base = ctx.fontBaseHelvetica10;
    else if (font == GLUT_BITMAP_HELVETICA_12) base = ctx.fontBaseHelvetica12;
    else if (font == GLUT_BITMAP_HELVETICA_18) base = ctx.fontBaseHelvetica18;
    else if (font == GLUT_BITMAP_TIMES_ROMAN_10) base = ctx.fontBaseTimes10;
    else if (font == GLUT_BITMAP_TIMES_ROMAN_24) base = ctx.fontBaseTimes24;
    else if (font == GLUT_BITMAP_8_BY_13)      base = ctx.fontBaseFixed8x13;
    else if (font == GLUT_BITMAP_9_BY_15)      base = ctx.fontBaseFixed9x15;

    if (base != 0) {
        glListBase(base);
        GLubyte c = (GLubyte)character;
        glCallLists(1, GL_UNSIGNED_BYTE, &c);
    }
}

inline int glutBitmapWidth(void* font, int character) {
    (void)character;
    if (font == GLUT_BITMAP_HELVETICA_10) return 6;
    if (font == GLUT_BITMAP_HELVETICA_12) return 8;
    if (font == GLUT_BITMAP_HELVETICA_18) return 11;
    if (font == GLUT_BITMAP_TIMES_ROMAN_10) return 6;
    if (font == GLUT_BITMAP_TIMES_ROMAN_24) return 14;
    if (font == GLUT_BITMAP_8_BY_13) return 8;
    if (font == GLUT_BITMAP_9_BY_15) return 9;
    return 8;
}

inline int glutBitmapLength(void* font, const unsigned char* string) {
    if (!string) return 0;
    int len = 0;
    for (const unsigned char* p = string; *p; p++) {
        len += glutBitmapWidth(font, *p);
    }
    return len;
}

inline void glutStrokeCharacter(void* font, int character) {
    (void)font;
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    if (character < 0 || character > 255) return;

    if (ctx.fontBaseStroke != 0) {
        // Draw character outline
        glCallList(ctx.fontBaseStroke + (unsigned int)character);
    } else {
        // Fallback translation
        glTranslatef(60.0f, 0.0f, 0.0f);
    }
}

inline int glutStrokeLength(void* font, const unsigned char* string) {
    (void)font;
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    if (!string) return 0;
    float total = 0.0f;
    for (const unsigned char* p = string; *p; p++) {
        unsigned char c = *p;
        if (ctx.fontBaseStroke != 0 && ctx.strokeMetrics[c].gmfCellIncX > 0.0f) {
            total += ctx.strokeMetrics[c].gmfCellIncX * 119.0f;
        } else {
            total += 60.0f;
        }
    }
    return (int)total;
}

inline int glutStrokeWidth(void* font, int character) {
    (void)font;
    NativeGLUT::Context& ctx = NativeGLUT::getContext();
    if (character >= 0 && character < 255 && ctx.strokeMetrics[character].gmfCellIncX > 0.0f) {
        return (int)(ctx.strokeMetrics[character].gmfCellIncX * 119.0f);
    }
    return 60;
}

#endif // __NATIVE_GLUT_H__

#pragma once
// ============================================================
// Button.h  —  SmartRoute AI Reusable UI Components & Helpers
//   Immediate-mode OpenGL UI widgets, panels, and text rendering
//   Coordinate system: y=0 at top, increasing downward (glOrtho)
// ============================================================
#include <GL/glut.h>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>

// ── Basic Drawing Primitives ─────────────────────────────────

inline void drawRect(float x, float y, float w, float h,
                     float r, float g, float b) {
    glColor3f(r, g, b);
    glBegin(GL_QUADS);
    glVertex2f(x,     y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x,     y + h);
    glEnd();
}

inline void drawPanel(float x, float y, float w, float h,
                      const float* bgCol, const float* borderCol = nullptr,
                      float borderWidth = 1.0f) {
    if (bgCol) {
        glColor3fv(bgCol);
        glBegin(GL_QUADS);
        glVertex2f(x,     y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x,     y + h);
        glEnd();
    }
    if (borderCol && borderWidth > 0.f) {
        glColor3fv(borderCol);
        glLineWidth(borderWidth);
        glBegin(GL_LINE_LOOP);
        glVertex2f(x,     y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x,     y + h);
        glEnd();
        glLineWidth(1.0f);
    }
}

inline void drawSep(float x, float y, float w,
                    float r = 0.20f, float g = 0.24f, float b = 0.33f) {
    glColor3f(r, g, b);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2f(x,     y);
    glVertex2f(x + w, y);
    glEnd();
}

inline void drawBitmapText(float x, float y, const char* text,
                           void* font = GLUT_BITMAP_HELVETICA_12) {
    glRasterPos2f(x, y);
    for (const char* c = text; *c; c++)
        glutBitmapCharacter(font, *c);
}

inline void drawLabel(float x, float y, const char* text,
                      void* font = GLUT_BITMAP_HELVETICA_12,
                      const float* col = nullptr) {
    if (col) glColor3fv(col);
    glRasterPos2f(x, y);
    for (const char* c = text; *c; c++)
        glutBitmapCharacter(font, *c);
}

inline void drawCircle(float cx, float cy, float r, int segments = 24) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= segments; i++) {
        float a = 6.2831853f * (float)i / (float)segments;
        glVertex2f(cx + r * cosf(a), cy + r * sinf(a));
    }
    glEnd();
}

inline void drawDiamond(float cx, float cy, float r) {
    glBegin(GL_QUADS);
    glVertex2f(cx,     cy - r);
    glVertex2f(cx + r, cy);
    glVertex2f(cx,     cy + r);
    glVertex2f(cx - r, cy);
    glEnd();
}

// ── Reusable Section, Metric & Phase Layout Helpers ──────────

inline void drawSectionHeader(float x, float y, const char* title,
                              const float* col = nullptr) {
    if (col) glColor3fv(col);
    else glColor3f(0.35f, 0.65f, 0.95f); // consistent blue accent
    drawBitmapText(x, y, title, GLUT_BITMAP_HELVETICA_10);
}

inline void drawTextTruncated(float x, float y, const std::string& text, float maxW,
                              void* font = GLUT_BITMAP_HELVETICA_12,
                              const float* col = nullptr) {
    if (col) glColor3fv(col);
    int fullW = glutBitmapLength(font, (const unsigned char*)text.c_str());
    if ((float)fullW <= maxW || maxW <= 10.f) {
        drawBitmapText(x, y, text.c_str(), font);
        return;
    }
    std::string s = text;
    int dotW = glutBitmapLength(font, (const unsigned char*)"...");
    while (!s.empty() && (float)(glutBitmapLength(font, (const unsigned char*)s.c_str()) + dotW) > maxW) {
        s.pop_back();
    }
    s += "...";
    drawBitmapText(x, y, s.c_str(), font);
}

inline void drawMetricRow(float x, float y, float w,
                          const char* label, const char* value,
                          const float* labelCol, const float* valCol,
                          void* font = GLUT_BITMAP_HELVETICA_12) {
    if (valCol) glColor3fv(valCol);
    else glColor3f(0.95f, 0.96f, 0.98f);
    int valW = glutBitmapLength(font, (const unsigned char*)value);
    float valX = x + w - (float)valW;
    drawBitmapText(valX, y, value, font);

    float maxLblW = std::max(20.f, valX - x - 8.f);
    drawTextTruncated(x, y, label, maxLblW, font, labelCol);
}

inline void drawPhaseHeader(float x, float y, float w, const char* phaseNum, const char* title,
                            const char* desc1, const char* desc2 = nullptr) {
    float h = desc2 ? 50.f : 44.f;
    static const float cardBg[3] = {0.11f, 0.14f, 0.20f};
    static const float border[3] = {0.20f, 0.24f, 0.33f};
    drawPanel(x, y, w, h, cardBg, border, 1.0f);

    std::string fullTitle = std::string(phaseNum) + " -- " + std::string(title);
    glColor3f(0.35f, 0.65f, 0.95f);
    drawBitmapText(x + 10.f, y + 15.f, fullTitle.c_str(), GLUT_BITMAP_HELVETICA_12);

    glColor3f(0.60f, 0.64f, 0.72f);
    drawBitmapText(x + 10.f, y + 30.f, desc1, GLUT_BITMAP_HELVETICA_10);
    if (desc2) {
        drawBitmapText(x + 10.f, y + 43.f, desc2, GLUT_BITMAP_HELVETICA_10);
    }
}

inline void drawGridLegend(float x, float y, float w) {
    float h = 66.f;
    static const float cardBg[3] = {0.11f, 0.14f, 0.20f};
    static const float border[3] = {0.20f, 0.24f, 0.33f};
    drawPanel(x, y, w, h, cardBg, border, 1.0f);

    drawSectionHeader(x + 10.f, y + 14.f, "GRID LEGEND");

    auto drawLegendItem = [&](float ix, float iy, const float* col, const char* iconText, const char* label) {
        glColor3fv(col);
        glBegin(GL_QUADS);
        glVertex2f(ix, iy - 9.f);
        glVertex2f(ix + 14.f, iy - 9.f);
        glVertex2f(ix + 14.f, iy + 3.f);
        glVertex2f(ix, iy + 3.f);
        glEnd();
        if (iconText && *iconText) {
            glColor3f(1.f, 1.f, 1.f);
            drawBitmapText(ix + 3.f, iy + 1.f, iconText, GLUT_BITMAP_HELVETICA_10);
        }
        glColor3f(0.85f, 0.88f, 0.94f);
        drawBitmapText(ix + 20.f, iy + 1.f, label, GLUT_BITMAP_HELVETICA_10);
    };

    static const float colStart[3] = {0.10f, 0.75f, 0.38f};
    static const float colGoal[3]  = {0.15f, 0.45f, 0.88f};
    static const float colWall[3]  = {0.18f, 0.21f, 0.28f};
    static const float colRoad[3]  = {0.09f, 0.11f, 0.16f};

    drawLegendItem(x + 10.f,  y + 32.f, colStart, "S", "Store / Start");
    drawLegendItem(x + 145.f, y + 32.f, colGoal,  "C", "Customer (C1..C10)");
    drawLegendItem(x + 10.f,  y + 52.f, colWall,  "",  "Wall (Blocked)");
    drawLegendItem(x + 145.f, y + 52.f, colRoad,  "",  "Open Road");
}

inline void drawTerrainLegend(float x, float y, float w, bool includeCongested = true) {
    float h = 66.f;
    static const float cardBg[3] = {0.11f, 0.14f, 0.20f};
    static const float border[3] = {0.20f, 0.24f, 0.33f};
    drawPanel(x, y, w, h, cardBg, border, 1.0f);

    drawSectionHeader(x + 10.f, y + 14.f, "TERRAIN COST LEGEND");

    auto drawItem = [&](float ix, float iy, const char* label, const char* cost, const float* col) {
        glColor3fv(col);
        drawBitmapText(ix, iy, label, GLUT_BITMAP_HELVETICA_10);
        glColor3f(0.95f, 0.96f, 0.98f);
        drawBitmapText(ix + 62.f, iy, cost, GLUT_BITMAP_HELVETICA_10);
    };

    static const float colNorm[3]   = {0.60f, 0.64f, 0.72f};
    static const float colReward[3] = {0.12f, 0.68f, 0.38f};
    static const float colCong[3]   = {0.85f, 0.52f, 0.12f};
    static const float colPen[3]    = {0.82f, 0.22f, 0.22f};

    drawItem(x + 10.f,  y + 32.f, "Normal:",   "1.0",     colNorm);
    drawItem(x + 145.f, y + 32.f, "Reward:",   "0.5",     colReward);
    drawItem(x + 10.f,  y + 50.f, "Congested:","2.0",     colCong);
    drawItem(x + 145.f, y + 50.f, "Penalty:",  "4.0",     colPen);
}

// ── Button Widget ────────────────────────────────────────────

class Button {
public:
    float       x = 0, y = 0, w = 100, h = 30;
    std::string label;

    bool hovered  = false;
    bool selected = false;
    bool disabled = false;
    bool visible  = true;

    // Colors
    float colNormal  [3] = {0.14f, 0.17f, 0.23f};
    float colHover   [3] = {0.20f, 0.25f, 0.34f};
    float colSelected[3] = {0.16f, 0.38f, 0.72f};
    float colDisabled[3] = {0.10f, 0.12f, 0.16f};
    float colBorder  [3] = {0.26f, 0.30f, 0.40f};
    float colSelBorder[3]= {0.35f, 0.65f, 0.95f};
    float colText    [3] = {0.92f, 0.94f, 0.96f};
    float colDisText [3] = {0.40f, 0.44f, 0.50f};

    void* font = GLUT_BITMAP_HELVETICA_12;

    Button() = default;
    Button(float x, float y, float w, float h, const std::string& lbl)
        : x(x), y(y), w(w), h(h), label(lbl) {}

    // Style setters
    void setNeutral() {
        colNormal[0]=0.14f; colNormal[1]=0.17f; colNormal[2]=0.23f;
        colHover [0]=0.20f; colHover [1]=0.25f; colHover [2]=0.34f;
        colBorder[0]=0.26f; colBorder[1]=0.30f; colBorder[2]=0.40f;
        colSelected[0]=0.18f; colSelected[1]=0.24f; colSelected[2]=0.36f;
        colSelBorder[0]=0.35f; colSelBorder[1]=0.65f; colSelBorder[2]=0.95f;
    }
    void setPrimary() {
        colNormal[0]=0.14f; colNormal[1]=0.36f; colNormal[2]=0.72f;
        colHover [0]=0.20f; colHover [1]=0.46f; colHover [2]=0.88f;
        colBorder[0]=0.30f; colBorder[1]=0.55f; colBorder[2]=0.95f;
        colSelected[0]=0.18f; colSelected[1]=0.44f; colSelected[2]=0.85f;
        colSelBorder[0]=0.45f; colSelBorder[1]=0.75f; colSelBorder[2]=1.00f;
    }
    void setGreen() {
        colNormal[0]=0.08f; colNormal[1]=0.46f; colNormal[2]=0.24f;
        colHover [0]=0.12f; colHover [1]=0.58f; colHover [2]=0.32f;
        colBorder[0]=0.18f; colBorder[1]=0.70f; colBorder[2]=0.38f;
        colSelected[0]=0.12f; colSelected[1]=0.60f; colSelected[2]=0.32f;
        colSelBorder[0]=0.30f; colSelBorder[1]=0.95f; colSelBorder[2]=0.50f;
    }
    void setRed() {
        colNormal[0]=0.55f; colNormal[1]=0.16f; colNormal[2]=0.16f;
        colHover [0]=0.68f; colHover [1]=0.22f; colHover [2]=0.22f;
        colBorder[0]=0.80f; colBorder[1]=0.28f; colBorder[2]=0.28f;
        colSelected[0]=0.68f; colSelected[1]=0.20f; colSelected[2]=0.20f;
        colSelBorder[0]=0.98f; colSelBorder[1]=0.40f; colSelBorder[2]=0.40f;
    }
    void setAmber() {
        colNormal[0]=0.58f; colNormal[1]=0.36f; colNormal[2]=0.08f;
        colHover [0]=0.72f; colHover [1]=0.46f; colHover [2]=0.12f;
        colBorder[0]=0.85f; colBorder[1]=0.58f; colBorder[2]=0.16f;
        colSelected[0]=0.70f; colSelected[1]=0.45f; colSelected[2]=0.12f;
        colSelBorder[0]=0.98f; colSelBorder[1]=0.72f; colSelBorder[2]=0.20f;
    }
    void setGold() { setAmber(); }
    void setBlue() { setPrimary(); }

    void draw() const {
        if (!visible) return;

        // Background Fill
        if      (disabled) glColor3fv(colDisabled);
        else if (selected) glColor3fv(colSelected);
        else if (hovered)  glColor3fv(colHover);
        else               glColor3fv(colNormal);

        glBegin(GL_QUADS);
        glVertex2f(x,     y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x,     y + h);
        glEnd();

        // Border
        if (selected) {
            glColor3fv(colSelBorder);
            glLineWidth(2.5f);
        } else if (hovered) {
            glColor3f(colBorder[0] + 0.15f, colBorder[1] + 0.15f, colBorder[2] + 0.15f);
            glLineWidth(1.5f);
        } else {
            glColor3fv(colBorder);
            glLineWidth(1.0f);
        }
        glBegin(GL_LINE_LOOP);
        glVertex2f(x,     y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x,     y + h);
        glEnd();
        glLineWidth(1.0f);

        // Label Rendering with auto-scale to prevent clipping
        if (disabled) glColor3fv(colDisText);
        else if (selected) glColor3f(1.0f, 1.0f, 1.0f);
        else glColor3fv(colText);

        void* drawFont = font;
        int tw = glutBitmapLength(drawFont, (const unsigned char*)label.c_str());
        if (tw > (int)(w - 8.f) && drawFont == GLUT_BITMAP_HELVETICA_12) {
            drawFont = GLUT_BITMAP_HELVETICA_10;
            tw = glutBitmapLength(drawFont, (const unsigned char*)label.c_str());
        }

        float tx = std::max(x + 4.f, x + (w - (float)tw) * 0.5f);
        float ty = y + h * 0.5f + 4.f; // Optical vertical center

        glRasterPos2f(tx, ty);
        for (char c : label) glutBitmapCharacter(drawFont, c);
    }

    bool contains(float mx, float my) const {
        return mx >= x && mx <= x + w && my >= y && my <= y + h;
    }

    void updateHover(float mx, float my) {
        if (!visible || disabled) { hovered = false; return; }
        hovered = contains(mx, my);
    }

    bool isClicked(float mx, float my) const {
        return visible && !disabled && contains(mx, my);
    }
};

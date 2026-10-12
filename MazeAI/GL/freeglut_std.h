/*
 * freeglut_std.h — FreeGLUT Standard Definitions
 * Covers all symbols used in MazeAI (OpenGL + GLUT on Windows/MinGW).
 * Based on public FreeGLUT 3.x API (LGPL). Adapted as a self-contained header.
 */
#ifndef  __FREEGLUT_STD_H__
#define  __FREEGLUT_STD_H__

/* ── Platform setup ──────────────────────────────────────── */
#include <stdlib.h>

#if defined(_MSC_VER) || defined(__MINGW32__) || defined(__MINGW64__)
#   ifndef WIN32_LEAN_AND_MEAN
#       define WIN32_LEAN_AND_MEAN 1
#   endif
#   include <windows.h>
#   include <GL/gl.h>
#   include <GL/glu.h>
#   ifndef FGAPI
#       define FGAPI  __declspec(dllimport)
#   endif
#   ifndef FGAPIENTRY
#       define FGAPIENTRY __cdecl
#   endif
#else
#   include <GL/gl.h>
#   include <GL/glu.h>
#   ifndef FGAPI
#       define FGAPI
#   endif
#   ifndef FGAPIENTRY
#       define FGAPIENTRY
#   endif
#endif

/* ── GLUT API version ────────────────────────────────────── */
#define GLUT_API_VERSION                         4
#define GLUT_XLIB_IMPLEMENTATION                 13
#define FREEGLUT                                 1
#define FREEGLUT_VERSION_2_0                     1
#define GLUT_INIT_STATE                          0x007C

/* ── Display mode bits ───────────────────────────────────── */
#define GLUT_RGB                                 0x0000
#define GLUT_RGBA                                0x0000
#define GLUT_INDEX                               0x0001
#define GLUT_SINGLE                              0x0000
#define GLUT_DOUBLE                              0x0002
#define GLUT_ACCUM                               0x0004
#define GLUT_ALPHA                               0x0008
#define GLUT_DEPTH                               0x0010
#define GLUT_STENCIL                             0x0020
#define GLUT_MULTISAMPLE                         0x0080
#define GLUT_STEREO                              0x0100
#define GLUT_LUMINANCE                           0x0200

/* ── get()/device() constants ────────────────────────────── */
#define GLUT_WINDOW_X                            0x0064
#define GLUT_WINDOW_Y                            0x0065
#define GLUT_WINDOW_WIDTH                        0x0066
#define GLUT_WINDOW_HEIGHT                       0x0067
#define GLUT_WINDOW_BUFFER_SIZE                  0x0068
#define GLUT_WINDOW_STENCIL_SIZE                 0x0069
#define GLUT_WINDOW_DEPTH_SIZE                   0x006A
#define GLUT_WINDOW_RED_SIZE                     0x006B
#define GLUT_WINDOW_GREEN_SIZE                   0x006C
#define GLUT_WINDOW_BLUE_SIZE                    0x006D
#define GLUT_WINDOW_ALPHA_SIZE                   0x006E
#define GLUT_WINDOW_ACCUM_RED_SIZE               0x006F
#define GLUT_WINDOW_ACCUM_GREEN_SIZE             0x0070
#define GLUT_WINDOW_ACCUM_BLUE_SIZE              0x0071
#define GLUT_WINDOW_ACCUM_ALPHA_SIZE             0x0072
#define GLUT_WINDOW_DOUBLEBUFFER                 0x0073
#define GLUT_WINDOW_RGBA                         0x0074
#define GLUT_WINDOW_PARENT                       0x0075
#define GLUT_WINDOW_NUM_CHILDREN                 0x0076
#define GLUT_WINDOW_COLORMAP_SIZE                0x0077
#define GLUT_WINDOW_NUM_SAMPLES                  0x0078
#define GLUT_WINDOW_STEREO                       0x0079
#define GLUT_WINDOW_CURSOR                       0x007A
#define GLUT_SCREEN_WIDTH                        0x00C8
#define GLUT_SCREEN_HEIGHT                       0x00C9
#define GLUT_SCREEN_WIDTH_MM                     0x00CA
#define GLUT_SCREEN_HEIGHT_MM                    0x00CB
#define GLUT_MENU_NUM_ITEMS                      0x012C
#define GLUT_DISPLAY_MODE_POSSIBLE               0x0190
#define GLUT_INIT_WINDOW_X                       0x01F4
#define GLUT_INIT_WINDOW_Y                       0x01F5
#define GLUT_INIT_WINDOW_WIDTH                   0x01F6
#define GLUT_INIT_WINDOW_HEIGHT                  0x01F7
#define GLUT_INIT_DISPLAY_MODE                   0x01F8
#define GLUT_ELAPSED_TIME                        0x02BC
#define GLUT_WINDOW_FORMAT_ID                    0x007B
#define GLUT_INIT_STATE                          0x007C

/* ── Keyboard ────────────────────────────────────────────── */
#define GLUT_KEY_F1                              0x0001
#define GLUT_KEY_F2                              0x0002
#define GLUT_KEY_F3                              0x0003
#define GLUT_KEY_F4                              0x0004
#define GLUT_KEY_F5                              0x0005
#define GLUT_KEY_F6                              0x0006
#define GLUT_KEY_F7                              0x0007
#define GLUT_KEY_F8                              0x0008
#define GLUT_KEY_F9                              0x0009
#define GLUT_KEY_F10                             0x000A
#define GLUT_KEY_F11                             0x000B
#define GLUT_KEY_F12                             0x000C
#define GLUT_KEY_LEFT                            0x0064
#define GLUT_KEY_UP                              0x0065
#define GLUT_KEY_RIGHT                           0x0066
#define GLUT_KEY_DOWN                            0x0067
#define GLUT_KEY_PAGE_UP                         0x0068
#define GLUT_KEY_PAGE_DOWN                       0x0069
#define GLUT_KEY_HOME                            0x006A
#define GLUT_KEY_END                             0x006B
#define GLUT_KEY_INSERT                          0x006C

/* ── Mouse ───────────────────────────────────────────────── */
#define GLUT_LEFT_BUTTON                         0x0000
#define GLUT_MIDDLE_BUTTON                       0x0001
#define GLUT_RIGHT_BUTTON                        0x0002
#define GLUT_DOWN                                0x0000
#define GLUT_UP                                  0x0001
#define GLUT_LEFT                                0x0000
#define GLUT_ENTERED                             0x0001

/* ── Window state ────────────────────────────────────────── */
#define GLUT_HIDDEN                              0x0000
#define GLUT_FULLY_RETAINED                      0x0001
#define GLUT_PARTIALLY_RETAINED                  0x0002
#define GLUT_FULLY_COVERED                       0x0003
#define GLUT_NOT_VISIBLE                         0
#define GLUT_VISIBLE                             1
#define GLUT_NORMAL                              0x0000
#define GLUT_OVERLAY                             0x0001

/* ── Fonts ───────────────────────────────────────────────── */
#ifdef _MSC_VER
#pragma warning(disable:4152)   /* function-pointer cast */
#endif

FGAPI void* FGAPIENTRY __glutBitmapHelvetica10(void);
FGAPI void* FGAPIENTRY __glutBitmapHelvetica12(void);
FGAPI void* FGAPIENTRY __glutBitmapHelvetica18(void);
FGAPI void* FGAPIENTRY __glutBitmapTimesRoman10(void);
FGAPI void* FGAPIENTRY __glutBitmapTimesRoman24(void);
FGAPI void* FGAPIENTRY __glutBitmapFixed8x13(void);
FGAPI void* FGAPIENTRY __glutBitmapFixed9x15(void);
FGAPI void* FGAPIENTRY __glutStrokeRoman(void);
FGAPI void* FGAPIENTRY __glutStrokeMonoRoman(void);

#define GLUT_STROKE_ROMAN               ((void*)(&__glutStrokeRoman))
#define GLUT_STROKE_MONO_ROMAN          ((void*)(&__glutStrokeMonoRoman))
#define GLUT_BITMAP_9_BY_15             ((void*)(&__glutBitmapFixed9x15))
#define GLUT_BITMAP_8_BY_13             ((void*)(&__glutBitmapFixed8x13))
#define GLUT_BITMAP_TIMES_ROMAN_10      ((void*)(&__glutBitmapTimesRoman10))
#define GLUT_BITMAP_TIMES_ROMAN_24      ((void*)(&__glutBitmapTimesRoman24))
#define GLUT_BITMAP_HELVETICA_10        ((void*)(&__glutBitmapHelvetica10))
#define GLUT_BITMAP_HELVETICA_12        ((void*)(&__glutBitmapHelvetica12))
#define GLUT_BITMAP_HELVETICA_18        ((void*)(&__glutBitmapHelvetica18))

/* ── Cursor ──────────────────────────────────────────────── */
#define GLUT_CURSOR_RIGHT_ARROW         0x0000
#define GLUT_CURSOR_LEFT_ARROW          0x0001
#define GLUT_CURSOR_INFO                0x0002
#define GLUT_CURSOR_DESTROY             0x0003
#define GLUT_CURSOR_HELP                0x0004
#define GLUT_CURSOR_CYCLE               0x0005
#define GLUT_CURSOR_SPRAY               0x0006
#define GLUT_CURSOR_WAIT                0x0007
#define GLUT_CURSOR_TEXT                0x0008
#define GLUT_CURSOR_CROSSHAIR           0x0009
#define GLUT_CURSOR_UP_DOWN             0x000A
#define GLUT_CURSOR_LEFT_RIGHT          0x000B
#define GLUT_CURSOR_TOP_SIDE            0x000C
#define GLUT_CURSOR_BOTTOM_SIDE         0x000D
#define GLUT_CURSOR_LEFT_SIDE           0x000E
#define GLUT_CURSOR_RIGHT_SIDE          0x000F
#define GLUT_CURSOR_TOP_LEFT_CORNER     0x0010
#define GLUT_CURSOR_TOP_RIGHT_CORNER    0x0011
#define GLUT_CURSOR_BOTTOM_RIGHT_CORNER 0x0012
#define GLUT_CURSOR_BOTTOM_LEFT_CORNER  0x0013
#define GLUT_CURSOR_FULL_CROSSHAIR      0x0063
#define GLUT_CURSOR_NONE                0x0064
#define GLUT_CURSOR_INHERIT             0x0065

/* ── Game mode ───────────────────────────────────────────── */
#define GLUT_GAME_MODE_DISPLAY_CHANGED  0x0001
#define GLUT_GAME_MODE_POSSIBLE         0x0002
#define GLUT_GAME_MODE_WIDTH            0x0003
#define GLUT_GAME_MODE_HEIGHT           0x0004
#define GLUT_GAME_MODE_PIXEL_DEPTH      0x0005
#define GLUT_GAME_MODE_REFRESH_RATE     0x0006
#define GLUT_GAME_MODE_ACTIVE           0x0007

/* ══════════════════════════════════════════════════════════ */
/*  GLUT API function declarations                           */
/* ══════════════════════════════════════════════════════════ */
#ifdef __cplusplus
extern "C" {
#endif

/* Initialization */
FGAPI void    FGAPIENTRY glutInit(int* pargc, char** argv);
FGAPI void    FGAPIENTRY glutInitWindowPosition(int x, int y);
FGAPI void    FGAPIENTRY glutInitWindowSize(int width, int height);
FGAPI void    FGAPIENTRY glutInitDisplayMode(unsigned int displayMode);
FGAPI void    FGAPIENTRY glutInitDisplayString(const char* displayMode);

/* Event processing */
FGAPI void    FGAPIENTRY glutMainLoop(void);

/* Window management */
FGAPI int     FGAPIENTRY glutCreateWindow(const char* title);
FGAPI int     FGAPIENTRY glutCreateSubWindow(int window, int x, int y, int width, int height);
FGAPI void    FGAPIENTRY glutDestroyWindow(int window);
FGAPI void    FGAPIENTRY glutSetWindow(int window);
FGAPI int     FGAPIENTRY glutGetWindow(void);
FGAPI void    FGAPIENTRY glutSetWindowTitle(const char* title);
FGAPI void    FGAPIENTRY glutSetIconTitle(const char* title);
FGAPI void    FGAPIENTRY glutReshapeWindow(int width, int height);
FGAPI void    FGAPIENTRY glutPositionWindow(int x, int y);
FGAPI void    FGAPIENTRY glutShowWindow(void);
FGAPI void    FGAPIENTRY glutHideWindow(void);
FGAPI void    FGAPIENTRY glutIconifyWindow(void);
FGAPI void    FGAPIENTRY glutPushWindow(void);
FGAPI void    FGAPIENTRY glutPopWindow(void);
FGAPI void    FGAPIENTRY glutFullScreen(void);

/* Display */
FGAPI void    FGAPIENTRY glutPostWindowRedisplay(int window);
FGAPI void    FGAPIENTRY glutPostRedisplay(void);
FGAPI void    FGAPIENTRY glutSwapBuffers(void);

/* Mouse cursor */
FGAPI void    FGAPIENTRY glutWarpPointer(int x, int y);
FGAPI void    FGAPIENTRY glutSetCursor(int cursor);

/* Overlay */
FGAPI void    FGAPIENTRY glutEstablishOverlay(void);
FGAPI void    FGAPIENTRY glutRemoveOverlay(void);
FGAPI void    FGAPIENTRY glutUseLayer(GLenum layer);
FGAPI void    FGAPIENTRY glutPostOverlayRedisplay(void);
FGAPI void    FGAPIENTRY glutPostWindowOverlayRedisplay(int window);
FGAPI void    FGAPIENTRY glutShowOverlay(void);
FGAPI void    FGAPIENTRY glutHideOverlay(void);

/* Menu */
FGAPI int     FGAPIENTRY glutCreateMenu(void (* callback)(int menu));
FGAPI void    FGAPIENTRY glutDestroyMenu(int menu);
FGAPI int     FGAPIENTRY glutGetMenu(void);
FGAPI void    FGAPIENTRY glutSetMenu(int menu);
FGAPI void    FGAPIENTRY glutAddMenuEntry(const char* label, int value);
FGAPI void    FGAPIENTRY glutAddSubMenu(const char* label, int subMenu);
FGAPI void    FGAPIENTRY glutChangeToMenuEntry(int item, const char* label, int value);
FGAPI void    FGAPIENTRY glutChangeToSubMenu(int item, const char* label, int value);
FGAPI void    FGAPIENTRY glutRemoveMenuItem(int item);
FGAPI void    FGAPIENTRY glutAttachMenu(int button);
FGAPI void    FGAPIENTRY glutDetachMenu(int button);

/* Callback registration */
FGAPI void    FGAPIENTRY glutTimerFunc(unsigned int time, void (* callback)(int), int value);
FGAPI void    FGAPIENTRY glutIdleFunc(void (* callback)(void));
FGAPI void    FGAPIENTRY glutKeyboardFunc(void (* callback)(unsigned char, int, int));
FGAPI void    FGAPIENTRY glutSpecialFunc(void (* callback)(int, int, int));
FGAPI void    FGAPIENTRY glutReshapeFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutVisibilityFunc(void (* callback)(int));
FGAPI void    FGAPIENTRY glutDisplayFunc(void (* callback)(void));
FGAPI void    FGAPIENTRY glutMouseFunc(void (* callback)(int, int, int, int));
FGAPI void    FGAPIENTRY glutMotionFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutPassiveMotionFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutEntryFunc(void (* callback)(int));
FGAPI void    FGAPIENTRY glutKeyboardUpFunc(void (* callback)(unsigned char, int, int));
FGAPI void    FGAPIENTRY glutSpecialUpFunc(void (* callback)(int, int, int));
FGAPI void    FGAPIENTRY glutWindowStatusFunc(void (* callback)(int));
FGAPI void    FGAPIENTRY glutOverlayDisplayFunc(void (* callback)(void));
FGAPI void    FGAPIENTRY glutSpaceballMotionFunc(void (* callback)(int, int, int));
FGAPI void    FGAPIENTRY glutSpaceballRotateFunc(void (* callback)(int, int, int));
FGAPI void    FGAPIENTRY glutSpaceballButtonFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutButtonBoxFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutDialsFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutTabletMotionFunc(void (* callback)(int, int));
FGAPI void    FGAPIENTRY glutTabletButtonFunc(void (* callback)(int, int, int, int));

/* State */
FGAPI int     FGAPIENTRY glutGet(GLenum query);
FGAPI int     FGAPIENTRY glutDeviceGet(GLenum query);
FGAPI int     FGAPIENTRY glutGetModifiers(void);
FGAPI int     FGAPIENTRY glutLayerGet(GLenum query);

/* Font */
FGAPI void    FGAPIENTRY glutBitmapCharacter(void* font, int character);
FGAPI int     FGAPIENTRY glutBitmapWidth(void* font, int character);
FGAPI void    FGAPIENTRY glutStrokeCharacter(void* font, int character);
FGAPI int     FGAPIENTRY glutStrokeWidth(void* font, int character);
FGAPI int     FGAPIENTRY glutBitmapLength(void* font, const unsigned char* string);
FGAPI int     FGAPIENTRY glutStrokeLength(void* font, const unsigned char* string);

/* Geometric shapes */
FGAPI void    FGAPIENTRY glutWireSphere(GLdouble radius, GLint slices, GLint stacks);
FGAPI void    FGAPIENTRY glutSolidSphere(GLdouble radius, GLint slices, GLint stacks);
FGAPI void    FGAPIENTRY glutWireCone(GLdouble base, GLdouble height, GLint slices, GLint stacks);
FGAPI void    FGAPIENTRY glutSolidCone(GLdouble base, GLdouble height, GLint slices, GLint stacks);
FGAPI void    FGAPIENTRY glutWireBox(GLdouble width, GLdouble height, GLdouble depth);
FGAPI void    FGAPIENTRY glutSolidBox(GLdouble width, GLdouble height, GLdouble depth);
FGAPI void    FGAPIENTRY glutWireTorus(GLdouble innerRadius, GLdouble outerRadius, GLint sides, GLint rings);
FGAPI void    FGAPIENTRY glutSolidTorus(GLdouble innerRadius, GLdouble outerRadius, GLint sides, GLint rings);
FGAPI void    FGAPIENTRY glutWireDodecahedron(void);
FGAPI void    FGAPIENTRY glutSolidDodecahedron(void);
FGAPI void    FGAPIENTRY glutWireTeapot(GLdouble size);
FGAPI void    FGAPIENTRY glutSolidTeapot(GLdouble size);
FGAPI void    FGAPIENTRY glutWireOctahedron(void);
FGAPI void    FGAPIENTRY glutSolidOctahedron(void);
FGAPI void    FGAPIENTRY glutWireTetrahedron(void);
FGAPI void    FGAPIENTRY glutSolidTetrahedron(void);
FGAPI void    FGAPIENTRY glutWireIcosahedron(void);
FGAPI void    FGAPIENTRY glutSolidIcosahedron(void);

/* Video resize */
FGAPI int     FGAPIENTRY glutVideoResizeGet(GLenum query);
FGAPI void    FGAPIENTRY glutSetupVideoResizing(void);
FGAPI void    FGAPIENTRY glutStopVideoResizing(void);
FGAPI void    FGAPIENTRY glutVideoResize(int x, int y, int width, int height);
FGAPI void    FGAPIENTRY glutVideoPan(int x, int y, int width, int height);

/* Color index */
FGAPI void    FGAPIENTRY glutSetColor(int color, GLfloat red, GLfloat green, GLfloat blue);
FGAPI GLfloat FGAPIENTRY glutGetColor(int color, int component);
FGAPI void    FGAPIENTRY glutCopyColormap(int window);

/* Misc */
FGAPI void    FGAPIENTRY glutIgnoreKeyRepeat(int ignore);
FGAPI void    FGAPIENTRY glutSetKeyRepeat(int repeatMode);
FGAPI void    FGAPIENTRY glutForceJoystickFunc(void);
FGAPI void    FGAPIENTRY glutReportErrors(void);

#ifdef __cplusplus
}
#endif

#endif /* __FREEGLUT_STD_H__ */

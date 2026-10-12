/*
 * freeglut_ext.h  —  FreeGLUT extension API (minimal, MazeAI-safe)
 * Declares only commonly used extensions, avoiding any duplicates.
 */
#ifndef  __FREEGLUT_EXT_H__
#define  __FREEGLUT_EXT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ── Action on window close ── */
#define GLUT_ACTION_EXIT                         0
#define GLUT_ACTION_GLUTMAINLOOP_RETURNS         1
#define GLUT_ACTION_CONTINUE_EXECUTION           2
#define GLUT_ACTION_ON_WINDOW_CLOSE              0x01F9

/* ── Window creation flags ── */
#define GLUT_CREATE_NEW_CONTEXT                  0
#define GLUT_USE_CURRENT_CONTEXT                 1

/* ── Context profile ── */
#define GLUT_CORE_PROFILE                        0x0001
#define GLUT_COMPATIBILITY_PROFILE               0x0002

/* ── Extension functions used by MazeAI ── */
FGAPI void   FGAPIENTRY glutMainLoopEvent(void);
FGAPI void   FGAPIENTRY glutLeaveMainLoop(void);
FGAPI void   FGAPIENTRY glutMouseWheelFunc(void (* callback)(int, int, int, int));
FGAPI void   FGAPIENTRY glutSetOption(GLenum option_flag, int value);
FGAPI void   FGAPIENTRY glutCloseFunc(void (* callback)(void));
FGAPI void   FGAPIENTRY glutWMCloseFunc(void (* callback)(void));

/* ── Font helpers ── */
FGAPI int     FGAPIENTRY glutBitmapHeight(void* font);
FGAPI GLfloat FGAPIENTRY glutStrokeHeight(void* font);
FGAPI void    FGAPIENTRY glutBitmapString(void* font, const unsigned char* string);
FGAPI void    FGAPIENTRY glutStrokeString(void* font, const unsigned char* string);

/* ── Solid/wire shapes (additional) ── */
FGAPI void FGAPIENTRY glutWireCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks);
FGAPI void FGAPIENTRY glutSolidCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks);

/* ── Full-screen toggle ── */
FGAPI void FGAPIENTRY glutFullScreenToggle(void);
FGAPI void FGAPIENTRY glutLeaveFullScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* __FREEGLUT_EXT_H__ */

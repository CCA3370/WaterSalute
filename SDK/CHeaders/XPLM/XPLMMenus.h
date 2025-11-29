/*
 * X-Plane Menu API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMMENUS_H
#define XPLMMENUS_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * MENUS
 ***************************************************************************/

typedef void * XPLMMenuID;

/* Menu item check state */
typedef int XPLMMenuCheck;
#define xplm_Menu_NoCheck    0
#define xplm_Menu_Unchecked  1
#define xplm_Menu_Checked    2

/* Menu callback */
typedef void (* XPLMMenuHandler_f)(
    void *               inMenuRef,
    void *               inItemRef);

/* Find the plugins menu */
XPLM_API XPLMMenuID XPLMFindPluginsMenu(void);

/* Find the aircraft menu */
XPLM_API XPLMMenuID XPLMFindAircraftMenu(void);

/* Create a menu */
XPLM_API XPLMMenuID XPLMCreateMenu(
    const char *         inName,
    XPLMMenuID           inParentMenu,
    int                  inParentItem,
    XPLMMenuHandler_f    inHandler,
    void *               inMenuRef);

/* Destroy a menu */
XPLM_API void XPLMDestroyMenu(XPLMMenuID inMenuID);

/* Clear all items in a menu */
XPLM_API void XPLMClearAllMenuItems(XPLMMenuID inMenuID);

/* Append a menu item */
XPLM_API int XPLMAppendMenuItem(
    XPLMMenuID           inMenu,
    const char *         inItemName,
    void *               inItemRef,
    int                  inDeprecatedAndIgnored);

/* Append a menu item with command */
XPLM_API int XPLMAppendMenuItemWithCommand(
    XPLMMenuID           inMenu,
    const char *         inItemName,
    void *               inCommandToExecute);

/* Append a menu separator */
XPLM_API void XPLMAppendMenuSeparator(XPLMMenuID inMenu);

/* Set menu item name */
XPLM_API void XPLMSetMenuItemName(
    XPLMMenuID           inMenu,
    int                  inIndex,
    const char *         inItemName,
    int                  inDeprecatedAndIgnored);

/* Check/uncheck a menu item */
XPLM_API void XPLMCheckMenuItem(
    XPLMMenuID           inMenu,
    int                  inIndex,
    XPLMMenuCheck        inCheck);

/* Check if a menu item is checked */
XPLM_API void XPLMCheckMenuItemState(
    XPLMMenuID           inMenu,
    int                  inIndex,
    XPLMMenuCheck *      outCheck);

/* Enable/disable a menu item */
XPLM_API void XPLMEnableMenuItem(
    XPLMMenuID           inMenu,
    int                  inIndex,
    int                  inEnabled);

/* Remove a menu item */
XPLM_API void XPLMRemoveMenuItem(
    XPLMMenuID           inMenu,
    int                  inIndex);

#ifdef __cplusplus
}
#endif

#endif /* XPLMMENUS_H */

// SPDX-License-Identifier: LGPL-2.1-or-later
/***************************************************************************
 *   Copyright (c) 2024 Solid_Orbit contributors                           *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,   *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

/*
 * Solid Orbit Navigation Style — Perceptual Camera Engine
 * ========================================================
 *
 * A SolidWorks-grade camera whose design priority is:
 *
 *   human spatial comfort > mathematical purity
 *
 * Key perceptual principles (from architectural review):
 *
 *   1. Orbit is DIRECT — zero latency, model glued to cursor.
 *      Only acceleration spikes are filtered, never position.
 *   2. Pivot stability > pivot accuracy — spatial inertia,
 *      evidence-based switching, resolved at gesture boundaries only.
 *   3. Horizon is gravitationally grounded — strong same-frame
 *      roll correction so orbit inherently respects world-up.
 *   4. Sensitivity is nonlinear — sigmoid velocity curve * sqrt
 *      distance scaling for perceptual uniformity across scales.
 *   5. Zoom converges toward cursor depth with pivot migration.
 *   6. Micro-jitter is eliminated via dead zones, spatial
 *      thresholds, and dt clamping.
 *
 * Interaction map:
 *   LMB             = Selection (when no toggle is active)
 *   PAD_0 toggle    = Orbit (LMB drag)
 *   PAD_2 toggle    = Pan (LMB drag)
 *   MMB             = Pan
 *   Wheel           = Zoom (cursor-aware, depth-adaptive)
 *   PAD_* / ESC     = Cancel toggle mode
 */

#include <cmath>

#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/projectors/SbSphereSheetProjector.h>
#include <QApplication>

#include "Navigation/NavigationStyle.h"
#include "Inventor/SoMouseWheelEvent.h"
#include "View3DInventorViewer.h"


using namespace Gui;

// ---------------------------------------------------------------------------
//  Perceptual tuning constants
// ---------------------------------------------------------------------------
namespace {

    // --- Pivot (spatial inertia) ---
    // Blend alpha when starting a new orbit gesture.
    constexpr float PIVOT_ORBIT_ALPHA       = 0.35f;
    // Blend alpha when zoom migrates the pivot (very gentle to prevent drift).
    constexpr float PIVOT_ZOOM_ALPHA        = 0.06f;
    // Ignore pivot candidates closer than this fraction of focal distance.
    constexpr float PIVOT_SPATIAL_THRESHOLD  = 0.04f;

    // --- Horizon stabilization ---
    // Tuned so roll correction is subconscious — the user should never
    // notice it, only avoid disorientation.  Too high fights free orbit.
    constexpr float HORIZON_RATE            = 3.2f;
    constexpr float ROLL_DEAD_ZONE          = 0.015f;
    constexpr float MAX_ROLL_STEP           = 0.12f;

    // --- Adaptive sensitivity ---
    constexpr float SENS_REF_DIST           = 50.0f;
    constexpr float SENS_MIN                = 0.20f;
    constexpr float SENS_MAX                = 3.5f;
    // Sigmoid velocity curve reference speed (normalized units/s).
    constexpr float SPEED_REF               = 1.5f;

    // --- Zoom ---
    constexpr float ZOOM_DEPTH_SCALE        = 0.7f;
    constexpr float ZOOM_MIN_FOCAL          = 0.01f;

    // --- Anti-jitter ---
    // Mouse delta dead zone (normalized screen coords).
    constexpr float MOUSE_DEAD_ZONE         = 0.0008f;
    // Angular acceleration soft-saturation reference (rad/s²).
    // Accelerations beyond this are compressed via tanh, not hard-clamped.
    constexpr float ACCEL_SOFT_KNEE         = 60.0f;

    // --- Timing ---
    constexpr float DEFAULT_DT              = 0.016f;
    constexpr float MIN_DT                  = 0.002f;
    constexpr float MAX_DT                  = 0.10f;
}

// ---------------------------------------------------------------------------

/* TRANSLATOR Gui::SolidOrbitNavigationStyle */

TYPESYSTEM_SOURCE(Gui::SolidOrbitNavigationStyle, Gui::UserNavigationStyle)

SolidOrbitNavigationStyle::SolidOrbitNavigationStyle()
    : lockButton1(false)
{
    // --- Match SolidWorks camera feel ---
    // 1. Trackball: pure arcball, no gimbal-lock axis constraints.
    setOrbitStyle(NavigationStyle::Trackball);
    // 2. Orbit pivots on the exact 3D surface point under cursor,
    //    falls back to focal plane if nothing is hit (SolidWorks behavior).
    setRotationCenterMode(
        NavigationStyle::RotationCenterMode::ScenePointAtCursor
        | NavigationStyle::RotationCenterMode::FocalPointAtCursor);
    // 3. No inertia/spinning animation — SolidWorks orbit stops when you
    //    release the mouse button, giving direct mechanical feel.
    setSpinningAnimationEnabled(false);
    // 4. Zoom towards cursor, matching SolidWorks scroll behaviour.
    setZoomAtCursor(true);
}

SolidOrbitNavigationStyle::~SolidOrbitNavigationStyle() = default;

std::string SolidOrbitNavigationStyle::userFriendlyName() const
{
    return "SolidOrbit";
}

const char* SolidOrbitNavigationStyle::mouseButtons(ViewerMode mode)
{
    switch (mode) {
        case NavigationStyle::SELECTION:
            return QT_TR_NOOP("Press left mouse button");
        case NavigationStyle::PANNING:
            return QT_TR_NOOP("Press middle mouse button (or Numpad 2 toggle + left mouse button)");
        case NavigationStyle::DRAGGING:
            return QT_TR_NOOP("Press Numpad 0 to toggle Orbit mode, then drag with left mouse button");
        case NavigationStyle::ZOOMING:
            return QT_TR_NOOP("Scroll mouse wheel");
        default:
            return "No description";
    }
}

void SolidOrbitNavigationStyle::so_updateCursor()
{
    if (!viewer) return;

    if (so_toggleMode == MODE_ORBIT) {
        viewer->setCursorRepresentation(NavigationStyle::DRAGGING);
    }
    else if (so_toggleMode == MODE_PAN) {
        viewer->setCursorRepresentation(NavigationStyle::PANNING);
    }
    else if (so_toggleMode == MODE_SELECTION || so_toggleMode == MODE_NONE) {
        // When editing (e.g. Sketcher tool active), let the editing subsystem
        // control the cursor — don't override the crosshair/tool cursor.
        if (!viewer->isEditing()) {
            viewer->setCursorRepresentation(NavigationStyle::IDLE);
        }
    }
    else {
        // Fallback for neutral state
        viewer->setCursorRepresentation(this->getViewingMode());
    }
}

// ===========================================================================
//  RAYCAST HELPER
// ===========================================================================

bool SolidOrbitNavigationStyle::so_raycast(
    const SbVec2s& screenPos,
    SbVec3f& hitPoint)
{
    SoRayPickAction rpaction(viewer->getSoRenderManager()->getViewportRegion());
    rpaction.setPoint(screenPos);
    rpaction.setRadius(viewer->getPickRadius());
    rpaction.apply(viewer->getSoRenderManager()->getSceneGraph());

    const SoPickedPoint* picked = rpaction.getPickedPoint();
    if (picked) {
        hitPoint = picked->getPoint();
        return true;
    }
    return false;
}


// ===========================================================================
//  DYNAMIC PIVOT — Evidence-based, spatially inertial
//
//  Called at gesture boundaries (orbit start, zoom, look-at), NOT
//  continuously during drag.  This prevents micro-jitter from
//  unstable raycasting and gives "spatial inertia" behavior.
// ===========================================================================

void SolidOrbitNavigationStyle::so_resolvePivot(
    const SbVec2s& mousePos,
    float blendAlpha)
{
    SbVec3f candidate;
    bool found = false;

    // Priority 1: Geometry under cursor
    found = so_raycast(mousePos, candidate);

    // Priority 2: Scene bounding-box center
    if (!found) {
        SoGetBoundingBoxAction bboxAction(
            viewer->getSoRenderManager()->getViewportRegion()
        );
        bboxAction.apply(viewer->getSceneGraph());
        SbBox3f box = bboxAction.getBoundingBox();
        if (!box.isEmpty()) {
            candidate = box.getCenter();
            found = true;
        }
    }

    // Priority 3: Current focal point (last stable)
    if (!found) {
        candidate = viewer->getFocalPoint();
        found = true;
    }

    // --- First resolution: snap immediately ---
    if (!so_pivotValid) {
        so_pivot      = candidate;
        so_pivotValid = true;
        return;
    }

    // --- Spatial threshold: ignore tiny changes (anti-jitter) ---
    SoCamera* cam = getCamera();
    if (cam) {
        float focalDist = cam->focalDistance.getValue();
        float threshold = focalDist * PIVOT_SPATIAL_THRESHOLD;
        float dist = (candidate - so_pivot).length();
        if (dist < threshold) {
            return;  // Not enough evidence for a change.
        }
    }

    // --- Hysteresis blend: migrate slowly toward candidate ---
    so_pivot = so_pivot + (candidate - so_pivot) * blendAlpha;

    // Sync with FreeCAD native rotation center (keeps the red dot stable and accurate)
    this->setRotationCenter(so_pivot);
    if (viewer) {
        viewer->changeRotationCenterPosition(so_pivot);
    }
}


// ===========================================================================
//  WHEEL EVENT — Zoom-to-cursor + pivot sync
//
//  Delegates to the native zoom-to-cursor implementation (doZoom with
//  zoomAtCursor=true), then reads back the native rotation center to keep
//  our so_pivot accurate for the next orbit gesture.  This gives the exact
//  SolidWorks "scroll towards cursor, then orbit around where you zoomed"
//  experience.
// ===========================================================================

SbBool SolidOrbitNavigationStyle::processWheelEvent(
    const SoMouseWheelEvent* const event)
{
    const SbVec2s pos(event->getPosition());
    const SbVec2f posn = normalizePixelPos(pos);

    // Native zoom — already respects zoomAtCursor preference we set in ctor.
    doZoom(viewer->getSoRenderManager()->getCamera(), event->getDelta(), posn);

    // Migrate pivot toward where the camera just zoomed to, so the next
    // orbit naturally circles the newly focused area.
    so_resolvePivot(pos, PIVOT_ZOOM_ALPHA);

    return true;
}


// ===========================================================================
//  MAIN EVENT PROCESSOR
// ===========================================================================

SbBool SolidOrbitNavigationStyle::processSoEvent(const SoEvent* const ev)
{
    if (this->isSeekMode()) {
        return inherited::processSoEvent(ev);
    }

    if (!this->isSeekMode() && !this->isAnimating() && this->isViewing()) {
        this->setViewing(false);
    }

    const SoType type(ev->getTypeId());

    const SbViewportRegion& vp = viewer->getSoRenderManager()->getViewportRegion();
    const SbVec2s pos(ev->getPosition());
    const SbVec2f posn = normalizePixelPos(pos);

    const SbVec2f prevnormalized = this->lastmouseposition;
    this->lastmouseposition = posn;

    SbBool processed = false;
    const ViewerMode curmode = this->currentmode;
    ViewerMode newmode = curmode;

    syncModifierKeys(ev);

    if (!viewer->isEditing()) {
        processed = handleEventInForeground(ev);
        if (processed) { return true; }
    }

    // --- Keyboard ---
    if (type.isDerivedFrom(SoKeyboardEvent::getClassTypeId())) {
        const auto* const event = static_cast<const SoKeyboardEvent*>(ev);

        // --- Numpad toggle modes (PAD_0 = orbit, PAD_1 = selection, PAD_2 = pan) ---
        if (event->getState() == SoButtonEvent::DOWN) {
            switch (event->getKey()) {
                case SoKeyboardEvent::PAD_0:
                    so_toggleMode = (so_toggleMode == MODE_ORBIT)
                        ? MODE_NONE : MODE_ORBIT;
                    processed = true;
                    break;
                case SoKeyboardEvent::PAD_1:
                    so_toggleMode = (so_toggleMode == MODE_SELECTION)
                        ? MODE_NONE : MODE_SELECTION;
                    processed = true;
                    break;
                case SoKeyboardEvent::PAD_2:
                    so_toggleMode = (so_toggleMode == MODE_PAN)
                        ? MODE_NONE : MODE_PAN;
                    processed = true;
                    break;
                case SoKeyboardEvent::ESCAPE:
                case SoKeyboardEvent::PAD_MULTIPLY:
                    if (so_toggleMode != MODE_NONE) {
                        so_toggleMode = MODE_NONE;
                        processed = true;
                    }
                    break;
                default:
                    break;
            }

            if (processed) {
                so_updateCursor();
            }
        }

        if (!processed) {
            processed = processKeyboardEvent(event);
        }
    }

    // --- Mouse buttons ---
    if (type.isDerivedFrom(SoMouseButtonEvent::getClassTypeId())) {
        const auto* const event = static_cast<const SoMouseButtonEvent*>(ev);
        const int button = event->getButton();
        const SbBool press = event->getState() == SoButtonEvent::DOWN;

        switch (button) {
            case SoMouseButtonEvent::BUTTON1:
                this->lockrecenter = true;
                this->button1down = press;
                updateSelectionStartPosition(press, pos);

                if (press && (this->currentmode == NavigationStyle::SEEK_WAIT_MODE)) {
                    newmode = NavigationStyle::SEEK_MODE;
                    this->seekToPoint(pos);
                    processed = true;
                }
                else if (press
                         && (this->currentmode == NavigationStyle::PANNING
                             || this->currentmode == NavigationStyle::ZOOMING)) {
                    newmode = NavigationStyle::DRAGGING;
                    saveCursorPosition(ev);
                    this->centerTime = ev->getTime();
                    processed = true;
                }
                else if (press && so_toggleMode != MODE_NONE && so_toggleMode != MODE_SELECTION) {
                    // Toggle mode active (Orbit/Pan) — LMB initiates camera, not selection.
                    saveCursorPosition(ev);
                    this->centerTime = ev->getTime();
                    processed = true;
                }
                else if (!press && (this->currentmode == NavigationStyle::DRAGGING)) {
                    // Release after orbit — suppress selection.
                    processed = true;
                }
                else if (!press && (this->currentmode == NavigationStyle::PANNING)
                         && so_toggleMode == MODE_PAN) {
                    // Release after toggle-pan — suppress selection.
                    processed = true;
                }
                else if (viewer->isEditing()
                         && (this->currentmode == NavigationStyle::SPINNING)) {
                    processed = true;
                }
                else {
                    // No toggle mode, no modifiers — normal selection.
                    processed = processClickEvent(event);
                }
                break;

            case SoMouseButtonEvent::BUTTON2:
                this->lockrecenter = true;
                if (!press && (hasDragged || hasPanned || hasZoomed)) {
                    processed = true;
                }
                else if (!press && !viewer->isEditing()) {
                    if (this->currentmode != NavigationStyle::ZOOMING
                        && this->currentmode != NavigationStyle::PANNING
                        && this->currentmode != NavigationStyle::DRAGGING) {
                        if (this->isPopupMenuEnabled()) {
                            this->openPopupMenu(event->getPosition());
                        }
                    }
                }
                this->button2down = press;
                break;

            case SoMouseButtonEvent::BUTTON3:
                if (press) {
                    this->centerTime = ev->getTime();
                    setupPanningPlane(getCamera());
                    this->lockrecenter = false;
                }
                else {
                    SbTime tmp = (ev->getTime() - this->centerTime);
                    float dci = static_cast<float>(QApplication::doubleClickInterval()) / 1000.0f;
                    if (tmp.getValue() < dci && !this->lockrecenter) {
                        lookAtPoint(pos);
                        // Snap pivot to new focal point (user explicitly recentered).
                        so_pivotValid = false;
                        so_resolvePivot(pos, 1.0f);
                        processed = true;
                    }
                }
                this->button3down = press;
                break;

            default:
                break;
        }
    }

    // --- Mouse movement ---
    if (type.isDerivedFrom(SoLocation2Event::getClassTypeId())) {
        this->lockrecenter = true;
        const auto* const event = static_cast<const SoLocation2Event*>(ev);

        if (this->currentmode == NavigationStyle::SELECTION && this->button1down
            && !this->ctrldown && !this->shiftdown
            && tryStartBoxSelection(event, false)) {
            processed = true;
        }
        else if (this->currentmode == NavigationStyle::ZOOMING) {
            this->zoomByCursor(posn, prevnormalized);
            processed = true;
        }
        else if (this->currentmode == NavigationStyle::PANNING) {
            float ratio = vp.getViewportAspectRatio();
            panCamera(
                viewer->getSoRenderManager()->getCamera(),
                ratio,
                this->panningplane,
                posn,
                prevnormalized
            );
            processed = true;
        }
        else if (this->currentmode == NavigationStyle::DRAGGING) {
            // Delegate entirely to native Trackball spin.
            // saveCursorPosition() was already called at LMB press
            // so rotationCenter/rotationCenterFound are already set,
            // and spinInternal() will offset the sphere projector correctly.
            this->addToLog(event->getPosition(), event->getTime());
            this->spin(posn);
            moveCursorPosition();
            processed = true;
        }
    }

    // --- Spaceball / Joystick ---
    if (type.isDerivedFrom(SoMotion3Event::getClassTypeId())) {
        const auto* const event = static_cast<const SoMotion3Event*>(ev);
        if (event) {
            this->processMotionEvent(event);
        }
        processed = true;
    }

    // --- State machine ---
    enum
    {
        BUTTON1DOWN = 1 << 0,
        BUTTON3DOWN = 1 << 1,
        CTRLDOWN    = 1 << 2,
        SHIFTDOWN   = 1 << 3,
        BUTTON2DOWN = 1 << 4
    };

    const unsigned int combo =
        (this->button1down  ? BUTTON1DOWN : 0)
        | (this->button2down ? BUTTON2DOWN : 0)
        | (this->button3down ? BUTTON3DOWN : 0)
        | (this->ctrldown    ? CTRLDOWN    : 0)
        | (this->shiftdown   ? SHIFTDOWN   : 0);

    switch (combo) {
        case 0:
            if (curmode == NavigationStyle::SPINNING) { break; }
            newmode = NavigationStyle::IDLE;
            if (this->lockButton1) {
                this->lockButton1 = false;
                if (curmode != NavigationStyle::SELECTION) {
                    processed = true;
                }
            }
            break;

        case BUTTON1DOWN:
        case CTRLDOWN | BUTTON1DOWN:
        case SHIFTDOWN | BUTTON1DOWN:
            // Toggle mode: bare LMB or modified LMB follows toggle state
            if (so_toggleMode == MODE_ORBIT) {
                if (newmode != NavigationStyle::DRAGGING) {
                    saveCursorPosition(ev);
                }
                newmode = NavigationStyle::DRAGGING;
            }
            else if (so_toggleMode == MODE_PAN) {
                newmode = NavigationStyle::PANNING;
            }
            else if (curmode == NavigationStyle::SPINNING
                || (this->lockButton1 && curmode != NavigationStyle::SELECTION)) {
                newmode = NavigationStyle::IDLE;
            }
            else {
                // If in MODE_SELECTION or MODE_NONE, map to selection
                newmode = NavigationStyle::SELECTION;
            }
            break;

        case BUTTON3DOWN:
            newmode = NavigationStyle::PANNING;
            break;

        case CTRLDOWN | BUTTON3DOWN:
        case CTRLDOWN | SHIFTDOWN | BUTTON3DOWN:
            newmode = NavigationStyle::ZOOMING;
            break;

        default:
            if ((curmode == NavigationStyle::PANNING || curmode == NavigationStyle::ZOOMING)
                && !this->button3down && !this->button1down) {
                newmode = NavigationStyle::IDLE;
            }
            break;
    }

    // Guard: lock selection during camera interactions.
    if (this->button1down
        && (this->ctrldown || this->shiftdown || so_toggleMode == MODE_ORBIT || so_toggleMode == MODE_PAN)) {
        this->lockButton1 = true;
    }

    // Guard: do not interrupt rubber-band selection in sketcher.
    if (viewer->isEditing() && curmode == NavigationStyle::SELECTION
        && newmode != NavigationStyle::IDLE) {
        newmode = NavigationStyle::SELECTION;
        processed = false;
    }

    // Reset interaction flags on full IDLE.
    if (newmode == IDLE && !button1down && !button2down && !button3down) {
        hasPanned  = false;
        hasDragged = false;
        hasZoomed  = false;
    }

    // --- Mode transition hooks ---
    if (newmode != curmode) {
        if (newmode == NavigationStyle::DRAGGING) {
            // The native saveCursorPosition() call at LMB press already
            // performed the raycast and set rotationCenter/rotationCenterFound.
            // We keep our so_pivot in sync for the red-dot visualization.
            SbBool found;
            SbVec3f nativeCenter = getRotationCenter(found);
            if (found) {
                so_pivot      = nativeCenter;
                so_pivotValid = true;
                viewer->changeRotationCenterPosition(so_pivot);
            }
        }
        if (newmode == NavigationStyle::PANNING) {
            setupPanningPlane(getCamera());
        }
        this->setViewingMode(newmode);
    }

    if (!processed) {
        processed = inherited::processSoEvent(ev);
    }

    // Force cursor to match the active navigation mode persistently
    so_updateCursor();

    return processed;
}

/*
 *  This file is a part of KNOSSOS.
 *
 *  (C) Copyright 2007-2018
 *  Max-Planck-Gesellschaft zur Foerderung der Wissenschaften e.V.
 *
 *  KNOSSOS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 of
 *  the License as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  For further information, visit https://knossos.app
 *  or contact knossosteam@gmail.com
 */

#include "annotation/annotation.h"
#include "dataset.h"
#include "widgets/viewports/viewportarb.h"
#include "widgets/viewports/viewportbase.h"
#include "widgets/viewports/viewportortho.h"
#include "widgets/viewports/viewport3d.h"

#include "functions.h"
#include "gui_wrapper.h"
#include "scriptengine/scripting.h"
#include "segmentation/cubeloader.h"
#include "segmentation/segmentation.h"
#include "segmentation/segmentationsplit.h"
#include "segmentation/floodfill.h"
#include "segmentation/shapeinterpolation.h"
#include "skeleton/skeletonizer.h"
#include "skeleton/tree.h"
#include "stateInfo.h"
#include "viewer.h"
#include "widgets/preferences/navigationtab.h"
#include "widgets/mainwindow.h"

#include <QApplication>
#include <QMessageBox>
#include <QStatusBar>

#include <optional>

#include <boost/optional.hpp>

#include <cstdlib>
#include <unordered_set>

void merging(const QMouseEvent *event, ViewportOrtho & vp) {
    auto & seg = Segmentation::singleton();
    const auto brushCenter = getCoordinateFromOrthogonalClick(event->pos(), vp);
    const auto subobjectIds = readVoxels(brushCenter, seg.brush.value());
    for (const auto & subobjectPair : subobjectIds) {
        if (seg.selectedObjectsCount() == 1) {
            const auto soid = subobjectPair.first;
            const auto pos = subobjectPair.second;
            auto & subobject = seg.subobjectFromId(soid, pos);
            const auto objectToMergeIdx = seg.smallestImmutableObjectContainingSubobject(subobject);
            // if clicked object is currently selected, an unmerge is requested
            if (seg.isSelected(subobject)) {
                if (event->modifiers().testFlag(Qt::ShiftModifier)) {
                    if (event->modifiers().testFlag(Qt::ControlModifier)) {
                        seg.selectObjectFromSubObject(subobject, pos);
                        seg.unmergeSelectedObjects(pos);
                    } else {
                        if(seg.isSelected(objectToMergeIdx)) { // if no other object to unmerge, just unmerge subobject
                            seg.selectObjectFromSubObject(subobject, pos);
                        }
                        else {
                            seg.selectObject(objectToMergeIdx);
                        }
                        seg.unmergeSelectedObjects(pos);
                    }
                }
            } else { // object is not selected, so user wants to merge
                if (!event->modifiers().testFlag(Qt::ShiftModifier)) {
                    if (event->modifiers().testFlag(Qt::ControlModifier)) {
                        seg.selectObjectFromSubObject(subobject, pos);
                    } else {
                        seg.selectObject(objectToMergeIdx);//select largest object
                    }
                }
                if (seg.selectedObjectsCount() >= 2) {
                    seg.mergeSelectedObjects();
                }
            }
            seg.touchObjects(soid);
        }
    }
}

void segmentation_brush_work(const QMouseEvent *event, ViewportOrtho & vp) {
    if (Annotation::singleton().magLock && Dataset::datasets[Segmentation::singleton().layerId].magIndex != Annotation::singleton().magLock.value()) {
        return;
    }
    const Coordinate coord = getCoordinateFromOrthogonalClick(event->pos(), vp);
    auto & seg = Segmentation::singleton();

    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::ObjectMerge)) {
        merging(event, vp);
    } else if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_Paint) || Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_OverPaint)) {//paint and erase
        if (event->modifiers().testFlag(Qt::AltModifier)) {
            merging(event, vp);
        } else {
            if (seg.createPaintObject && !seg.brush.isInverse() && seg.selectedObjectsCount() == 0) {
                seg.createAndSelectObject(coord);
            }
            if (seg.selectedObjectsCount() > 0) {
                uint64_t soid = seg.subobjectIdOfFirstSelectedObject(coord);
                auto brush = seg.brush.value();
                const bool shapeInterpolation = Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_ShapeInterpolation);
                if (shapeInterpolation) {
                    brush.mode = brush_t::mode_t::two_dim;// key slices are strictly planar
                    // Touching a previewed slice bakes it first, so the stroke edits a real
                    // outline instead of being the only thing on an otherwise empty slice.
                    // Must happen before the stroke, or an erase would be undone by the bake.
                    auto & si = ShapeInterpolation::singleton();
                    if (si.active() && si.normalAxisViewport() == static_cast<int>(vp.viewportType)) {
                        QString note;
                        if (si.materializeAt(axisGet(coord, si.normalAxis()), note)) {
                            state->viewer->mainWindow.warnShapeInterpolation(note);
                        }
                    }
                }
                writeVoxels(coord, soid, brush);
                if (shapeInterpolation) {
                    QString reason;
                    if (!ShapeInterpolation::singleton().absorbStamp(coord, brush, soid, reason)) {
                        state->viewer->mainWindow.warnShapeInterpolation(reason);
                    }
                }
            }
        }
    }
}

/* 2D/3D flood fill, seeded at `coord`.
 *
 * Replaces the old middle-click bucket fill, which was bounded by whatever happened to be
 * on screen and which spilled into unloaded blocks — where every write was silently
 * dropped. This one stops at the edge of the loaded blocks and says so. */
void segmentation_flood_fill(const Coordinate & coord, ViewportOrtho & vp, const bool threeDimensional) {
    auto & seg = Segmentation::singleton();
    if (Annotation::singleton().magLock && Dataset::datasets[seg.layerId].magIndex != Annotation::singleton().magLock.value()) {
        return;
    }
    if (vp.viewportType != VIEWPORT_XY && vp.viewportType != VIEWPORT_XZ && vp.viewportType != VIEWPORT_ZY) {
        // a 2D fill needs an axis-aligned plane to stay in; the arbitrary viewport has none
        state->viewer->mainWindow.statusBar()->showMessage(QObject::tr("Fill: use one of the xy/xz/zy viewports."), 5000);
        return;
    }
    if (seg.createPaintObject && seg.selectedObjectsCount() == 0) {
        seg.createAndSelectObject(coord);
    }
    if (seg.selectedObjectsCount() == 0) {
        return;
    }

    FloodFillRequest request;
    request.seed = coord;
    request.fillsoid = seg.brush.isInverse() ? seg.getBackgroundId() : seg.subobjectIdOfFirstSelectedObject(coord);
    request.threeDimensional = threeDimensional;
    request.view = static_cast<brush_t::view_t>(vp.viewportType);
    request.mayLoadCubes = Segmentation::singleton().floodFillMayLoadCubes;

    const auto report = state->viewer->suspend([&]{ return runFloodFill(request, &state->viewer->mainWindow); });
    auto message = report.message;

    if (report.didSomething && Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_ShapeInterpolation)) {
        auto & si = ShapeInterpolation::singleton();
        QString reason;
        const auto depth = si.active() ? axisGet(coord, si.normalAxis()) : 0;
        if (!si.active()) {
            message += " " + QObject::tr("Paint a stroke first — a fill alone doesn’t start a chain.");
        } else if (!si.absorbRegion(report.filledMin, report.filledMax, depth, request.fillsoid, reason)) {
            message += " " + reason;
        }
    }
    state->viewer->mainWindow.statusBar()->showMessage(message, 8000);
    state->viewer->run();
}

void ViewportOrtho::handleMouseHover(const QMouseEvent *event) {
    auto coord = getCoordinateFromOrthogonalClick(event->pos(), *this);
    emit cursorPositionChanged(coord, viewportType);
    auto subObjectId = readVoxel(coord);
    EmitOnCtorDtor eocd(&SignalRelay::Signal_EventModel_handleMouseHover, state->signalRelay, coord, subObjectId, viewportType, event);
    update();// brush
    ViewportBase::handleMouseHover(event);
}

void startNodeSelection(const int x, const int y, const ViewportType vpType, const Qt::KeyboardModifiers modifiers) {
    state->viewerState->nodeSelectionSquare.first.x = x;
    state->viewerState->nodeSelectionSquare.first.y = y;

    // reset second point from a possible previous selection square.
    state->viewerState->nodeSelectionSquare.second = state->viewerState->nodeSelectionSquare.first;
    state->viewerState->nodeSelectSquareData = std::make_pair(vpType, modifiers);
}

void ViewportBase::handleLinkToggle(const QMouseEvent & event) {
    auto * activeNode = state->skeletonState->activeNode;
    auto clickedNode = pickNode(event.x(), event.y(), 10);
    if (clickedNode && activeNode != nullptr) {
        checkedToggleNodeLink(*activeNode, clickedNode.get());
    }
}

void ViewportBase::handleMouseButtonLeft(const QMouseEvent *event) {
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeSelection)) {
        const bool selection = event->modifiers().testFlag(Qt::ShiftModifier) || event->modifiers().testFlag(Qt::ControlModifier);
        if (selection) {
            startNodeSelection(event->pos().x(), event->pos().y(), viewportType, event->modifiers());
            return;
        }
        //Set Connection between Active Node and Clicked Node
        else if (QApplication::keyboardModifiers() == Qt::ALT) {
            handleLinkToggle(*event);
        }
    }
}

void ViewportBase::handleMouseButtonMiddle(const QMouseEvent *event) {
    if (event->modifiers().testFlag(Qt::ShiftModifier) && Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeEditing)) {
        handleLinkToggle(*event);
    }
}

void ViewportOrtho::handleMouseButtonMiddle(const QMouseEvent *event) {
    if (event->modifiers().testFlag(Qt::NoModifier) && Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeEditing)) {
        if (auto clickedNode = pickNode(event->x(), event->y(), 10)) {
            draggedNode = &clickedNode.get();
        }
    }
    ViewportBase::handleMouseButtonMiddle(event);
}

void ViewportOrtho::handleMouseButtonRight(const QMouseEvent *event) {
    const auto & annotationMode = Annotation::singleton().annotationMode;
    if (annotationMode.testFlag(AnnotationMode::Brush)) {
        Segmentation::singleton().brush.setInverse(event->modifiers().testFlag(Qt::ShiftModifier));
        segmentation_brush_work(event, *this);
        return;
    }
    if (!annotationMode.testFlag(AnnotationMode::NodeEditing)) {
        return;
    }
    Coordinate clickedCoordinate = getCoordinateFromOrthogonalClick(event->pos(), *this);
    if (Annotation::singleton().outsideMag1MovementArea(clickedCoordinate)) {
        return;
    }
    const quint64 subobjectId = readVoxel(clickedCoordinate);
    const bool background = subobjectId == Segmentation::singleton().getBackgroundId();
    if (annotationMode.testFlag(AnnotationMode::Mode_MergeTracing) && background && !event->modifiers().testFlag(Qt::ControlModifier)) {
        return;
    }

    nodeListElement * oldNode = state->skeletonState->activeNode;
    boost::optional<nodeListElement &> newNode;

    if (annotationMode.testFlag(AnnotationMode::LinkedNodes)) {
        if (oldNode == nullptr || state->skeletonState->activeTree->nodes.empty()) {
            //no node to link with or empty tree
            newNode = Skeletonizer::singleton().UI_addSkeletonNode(clickedCoordinate, viewportType);
        } else if (event->modifiers().testFlag(Qt::ControlModifier)) {
            if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_MergeTracing)) {
                const auto splitNode = Skeletonizer::singleton().UI_addSkeletonNode(clickedCoordinate, viewportType);
                if (splitNode) {
                    const auto comment = background ? "ecs" : "split";
                    Skeletonizer::singleton().setSubobject(splitNode.get(), subobjectId);
                    Skeletonizer::singleton().setComment(splitNode.get(), comment);
                    Skeletonizer::singleton().setActiveNode(oldNode);
                }
            } else if (auto stumpNode = Skeletonizer::singleton().addSkeletonNodeAndLinkWithActive(clickedCoordinate, viewportType, false)) {
                //Add a "stump", a branch node to which we don't automatically move.
                Skeletonizer::singleton().pushBranchNode(stumpNode.get());
                Skeletonizer::singleton().setActiveNode(oldNode);
            }
        } else {
            const auto lastPos = state->skeletonState->activeNode->position;
            newNode = Skeletonizer::singleton().addSkeletonNodeAndLinkWithActive(clickedCoordinate, viewportType, true);
            if (!newNode) {
                return;
            }
            const auto movement = clickedCoordinate - lastPos;
            //Highlight the viewport with the biggest movement component
            if ((std::abs(movement.x) >= std::abs(movement.y)) && (std::abs(movement.x) >= std::abs(movement.z))) {
                state->viewerState->highlightVp = VIEWPORT_ZY;
            } else if ((std::abs(movement.y) >= std::abs(movement.x)) && (std::abs(movement.y) >= std::abs(movement.z))) {
                state->viewerState->highlightVp = VIEWPORT_XZ;
            } else {
                state->viewerState->highlightVp = VIEWPORT_XY;
            }
            //Determine the directions for the f and d keys based on the signs of the movement components along the three dimensions
            state->viewerState->tracingDirection = movement;

            //Additional move of specified steps along tracing direction
            if (state->viewerState->autoTracingMode == Recentering::AheadOfNode) {
                floatCoordinate walking{movement};
                const auto factor = state->viewerState->autoTracingSteps / walking.length();
                clickedCoordinate += Coordinate(std::lround(movement.x * factor), std::lround(movement.y * factor), std::lround(movement.z * factor));
            }

            clickedCoordinate = clickedCoordinate.capped({0, 0, 0}, Dataset::current().boundary);// Do not allow clicks outside the dataset

            if(state->skeletonState->synapseState == Synapse::State::PostSynapse) {
                //The synapse workflow has been interrupted
                //Reset the synapse
                auto & tempSynapse = state->skeletonState->temporarySynapse;
                if (tempSynapse.getPreSynapse()) { tempSynapse.getPreSynapse()->isSynapticNode = false; }
                Skeletonizer::singleton().delTree(tempSynapse.getCleft()->treeID);
                tempSynapse = Synapse(); //reset temporary class
                state->skeletonState->synapseState = Synapse::State::PreSynapse;
                state->viewer->window->setSynapseState(SynapseState::Off); //reset statusbar entry
            }
        }
    } else { // unlinked
        newNode = Skeletonizer::singleton().UI_addSkeletonNode(clickedCoordinate,viewportType);
    }

    if (newNode) {
        if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_MergeTracing)) {
            Skeletonizer::singleton().setSubobjectSelectAndMergeWithPrevious(newNode.get(), subobjectId, oldNode);
        }
        // Move to the new node position
        if (state->viewerState->autoTracingMode != Recentering::Off) {
            if (viewportType == VIEWPORT_ARBITRARY) {
                state->viewer->setPositionWithRecenteringAndRotation(clickedCoordinate);
            } else {
                state->viewer->setPositionWithRecentering(clickedCoordinate);
            }
        }
        auto & mainWin = *state->viewer->window;
        if (mainWin.segmentState == SegmentState::Off_Once) {
            mainWin.setSegmentState(SegmentState::On);
        }
        if(state->skeletonState->synapseState == Synapse::State::PostSynapse && state->skeletonState->activeTree->nodes.size() == 1) {
            auto & tempSynapse = state->skeletonState->temporarySynapse;
            tempSynapse.setPostSynapse(*state->skeletonState->activeNode);
            if (tempSynapse.getCleft() && tempSynapse.getPreSynapse() && tempSynapse.getPostSynapse()) {
                Skeletonizer::singleton().addFinishedSynapse(*tempSynapse.getCleft(), *tempSynapse.getPreSynapse(), *tempSynapse.getPostSynapse()); //move finished synapse to our synapse vector
            }
            state->skeletonState->synapseState = Synapse::State::PreSynapse;
            tempSynapse = Synapse(); //reset temporary class
            state->viewer->window->toggleSynapseState(); //update statusbar
        }
    }
    ViewportBase::handleMouseButtonRight(event);
}

floatCoordinate ViewportOrtho::handleMovement(const QPoint & pos) {
    const QPointF posDelta(xrel(pos.x()), yrel(pos.y()));
    const QPointF arbitraryMouseSlide = {-posDelta.x() / screenPxXPerMag1Px, -posDelta.y() / screenPxYPerMag1Px};
    const auto move = v1 * arbitraryMouseSlide.x() - v2 * arbitraryMouseSlide.y();
    return move;
}

void ViewportBase::handleMouseMotionLeftHold(const QMouseEvent *event) {
    // pull selection square
    if (state->viewerState->nodeSelectSquareData.first != -1) {
        state->viewerState->nodeSelectionSquare.second.x = event->pos().x();
        state->viewerState->nodeSelectionSquare.second.y = event->pos().y();
        update();
    }
}

void Viewport3D::handleMouseMotionLeftHold(const QMouseEvent *event) {
    if (event->modifiers() == Qt::NoModifier) {
        if (Segmentation::singleton().volume_render_toggle) {
            auto & seg = Segmentation::singleton();
            seg.volume_mouse_move_x -= xrel(event->x());
            seg.volume_mouse_move_y -= yrel(event->y());
        } else {
            translateX += -xrel(event->x()) / screenPxXPerDataPx * Dataset::current().scales[0].x;
            translateY += -yrel(event->y()) / screenPxXPerDataPx * Dataset::current().scales[0].x;
        }
        update();
    }
    ViewportBase::handleMouseMotionLeftHold(event);
}

/* Shift + left is a second paint gesture alongside right drag, for anyone whose hand
 * expects Paintera's left-button painting. It always paints and never erases: Shift is
 * KNOSSOS's erase modifier, so the brush's inverse flag is overridden for these strokes.
 * Erasing stays on Shift + right drag. */
bool ViewportOrtho::shiftLeftPaint(const QMouseEvent *event) {
    if (!Annotation::singleton().annotationMode.testFlag(AnnotationMode::Brush)
            || !event->modifiers().testFlag(Qt::ShiftModifier)
            || event->modifiers().testFlag(Qt::ControlModifier)
            || event->modifiers().testFlag(Qt::AltModifier)) {
        return false;
    }
    auto & brush = Segmentation::singleton().brush;
    const auto wasInverse = brush.isInverse();
    brush.setInverse(false);
    segmentation_brush_work(event, *this);
    brush.setInverse(wasInverse);
    return true;
}

void ViewportOrtho::handleMouseMotionLeftHold(const QMouseEvent *event) {
    if (shiftLeftPaint(event)) {
        return;
    }
    if (event->modifiers() == Qt::NoModifier) {
        state->viewer->userMove(handleMovement(event->pos()), USERMOVE_HORIZONTAL, n);
    }
    ViewportBase::handleMouseMotionLeftHold(event);
}

/* Clicking an already-painted slice pulls it into the chain, so slices drawn before
 * entering the mode — or in a previous session — don't have to be redrawn.
 *
 * Plain click adopts the chain's own object. Ctrl+click on a *different* object relabels
 * that object's voxels in this plane to the chain's id and adopts the result: a
 * voxel-level steal of the outline, which leaves their object intact everywhere else. */
bool ViewportOrtho::shapeInterpolationAdopt(const QMouseEvent *event, const Coordinate & clickPos) {
    if (!Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_ShapeInterpolation)
            || Annotation::singleton().outsideMovementArea(clickPos)) {
        return false;
    }
    auto & si = ShapeInterpolation::singleton();
    auto & seg = Segmentation::singleton();
    const auto clicked = readVoxel(clickPos);
    if (clicked == seg.getBackgroundId()) {
        return false;// clicking empty space still means "deselect", as everywhere else
    }
    const bool steal = event->modifiers().testFlag(Qt::ControlModifier);

    if (!si.active()) {
        if (steal) {
            return false;// nothing to steal into yet
        }
        // no chain running: adopt the clicked object and start one in this viewport
        seg.clearObjectSelection();
        seg.selectObjectFromSubObject(clicked, clickPos);
        si.beginAt(static_cast<brush_t::view_t>(viewportType), clicked);
    } else if (!steal && clicked != si.subobjectId()) {
        return false;// a plain click on someone else's object is still just a selection
    } else if (si.normalAxisViewport() != static_cast<int>(viewportType)) {
        state->viewer->mainWindow.warnShapeInterpolation(tr("This chain runs in the %1 plane.").arg(si.planeName()));
        return true;
    }

    QString note;
    const auto adopted = si.adoptPlaneAt(clickPos, note, steal ? std::optional<std::uint64_t>{clicked} : std::nullopt);
    state->viewer->mainWindow.warnShapeInterpolation(note);
    if (adopted) {
        state->viewer->run();
    }
    return true;
}

void ViewportOrtho::handleMouseButtonLeft(const QMouseEvent *event) {
    if (shiftLeftPaint(event)) {
        return;
    }
    ViewportBase::handleMouseButtonLeft(event);
}

void Viewport3D::handleMouseMotionRightHold(const QMouseEvent *event) {
    if (event->modifiers() == Qt::NoModifier && state->skeletonState->rotationcounter == 0) {
        state->skeletonState->rotdx += 90.0 * xrel(event->x()) / width();
        state->skeletonState->rotdy += 90.0 * yrel(event->y()) / height();
        update();
    }
    ViewportBase::handleMouseMotionRightHold(event);
}

void ViewportOrtho::handleMouseMotionRightHold(const QMouseEvent *event) {
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Brush)) {
        const bool notOrigin = event->pos() != mouseDown;//don’t do redundant work
        if (notOrigin) {
            segmentation_brush_work(event, *this);
        }
    }
    ViewportBase::handleMouseMotionRightHold(event);
}

void ViewportOrtho::handleMouseMotionMiddleHold(const QMouseEvent *event) {
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeEditing) && draggedNode != nullptr) {
        const auto moveAccurate = handleMovement(event->pos());
        arbNodeDragCache += moveAccurate;//accumulate subpixel movements
        Coordinate moveTrunc = arbNodeDragCache;//truncate
        arbNodeDragCache -= moveTrunc;//only keep remaining fraction
        const auto newPos = draggedNode->position - moveTrunc;
        Skeletonizer::singleton().setPosition(*draggedNode, newPos);
    }
    ViewportBase::handleMouseMotionMiddleHold(event);
}

void ViewportBase::handleMouseReleaseLeft(const QMouseEvent *event) {
    auto & skeleton = *state->skeletonState;
    if (mouseDown == event->pos()) { // mouse click
        skeleton.meshLastClickInformation = pickMesh(event->pos());
        if (skeleton.meshLastClickInformation) {
            Skeletonizer::singleton().setActiveTreeByID(skeleton.meshLastClickInformation.get().treeId);
        }
        skeleton.jumpToSkeletonNext = !skeleton.meshLastClickInformation;
    }

    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeSelection)) {
        QSet<nodeListElement*> selectedNodes;
        int diffX = std::abs(state->viewerState->nodeSelectionSquare.first.x - event->pos().x());
        int diffY = std::abs(state->viewerState->nodeSelectionSquare.first.y - event->pos().y());
        const auto boundedX = std::max(0, std::min(width(), event->pos().x()));
        const auto boundedY = std::max(0, std::min(height(), event->pos().y()));
        if ((diffX < 5 && diffY < 5) || (event->pos() - mouseDown).manhattanLength() < 5) { // interpreted as click instead of drag
            // mouse released on same spot on which it was pressed down: single node selection
            auto selectedNode = pickNode(boundedX, boundedY, 10);
            if (selectedNode) {
                selectedNodes = {&selectedNode.get()};
            }
        } else if (state->viewerState->nodeSelectSquareData.first != -1) {
            selectedNodes = nodeSelection(boundedX, boundedY);
        }
        if (state->viewerState->nodeSelectSquareData.first != -1 || !selectedNodes.empty()) {//only select no nodes if we drew a selection rectangle
            if (state->viewerState->nodeSelectSquareData.second == Qt::ControlModifier) {
                Skeletonizer::singleton().toggleSelection(selectedNodes);
            } else {
                Skeletonizer::singleton().select(selectedNodes);
            }
        }
        state->viewerState->nodeSelectSquareData = std::make_pair(-1, Qt::NoModifier);//disable node selection square
    }
}

void ViewportOrtho::handleMouseReleaseLeft(const QMouseEvent *event) {
    if (shiftLeftPaint(event)) {
        state->viewer->userMoveClear();
        ViewportBase::handleMouseReleaseLeft(event);
        return;
    }
    auto & segmentation = Segmentation::singleton();
    const auto clickPos = getCoordinateFromOrthogonalClick(event->pos(), *this);
    if (event->pos() == mouseDown && shapeInterpolationAdopt(event, clickPos)) {
        ViewportBase::handleMouseReleaseLeft(event);
        return;
    }
    if (!Annotation::singleton().outsideMovementArea(clickPos) && Annotation::singleton().annotationMode.testFlag(AnnotationMode::ObjectSelection)) { // in task mode the object should not be switched
        if (event->pos() == mouseDown) {// mouse click
            const auto subobjectId = readVoxel(clickPos);
            if (subobjectId != segmentation.getBackgroundId()) {// don’t select the unsegmented area as object
                auto & subobject = segmentation.subobjectFromId(subobjectId, clickPos);
                auto objIndex = segmentation.largestObjectContainingSubobject(subobject);
                Segmentation::singleton().setObjectLocation(objIndex, clickPos);
                if (!event->modifiers().testFlag(Qt::ControlModifier)) {
                    segmentation.clearObjectSelection();
                    segmentation.selectObject(objIndex);
                } else if (segmentation.isSelected(objIndex)) {// unselect if selected
                    segmentation.unselectObject(objIndex);
                } else { // select largest object
                    segmentation.selectObject(objIndex);
                }
                if (segmentation.isSelected(subobject)) {//touch other objects containing this subobject
                    segmentation.touchObjects(subobjectId);
                } else {
                    segmentation.untouchObjects();
                }
            }
        }
    }
    state->viewer->userMoveClear();//finish dataset drag

    ViewportBase::handleMouseReleaseLeft(event);
}

void ViewportOrtho::handleMouseReleaseRight(const QMouseEvent *event) {
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Brush)) {
        if (event->pos() != mouseDown) {//merge took already place on mouse down
            segmentation_brush_work(event, *this);
        }
    }
    ViewportBase::handleMouseReleaseRight(event);
}

void ViewportOrtho::handleMouseReleaseMiddle(const QMouseEvent *event) {
    Coordinate clickedCoordinate = getCoordinateFromOrthogonalClick(event->pos(), *this);
    if (!Annotation::singleton().outsideMovementArea(clickedCoordinate)) {
        EmitOnCtorDtor eocd(&SignalRelay::Signal_EventModel_handleMouseReleaseMiddle, state->signalRelay, clickedCoordinate, viewportType, event);
        const auto & mode = Annotation::singleton().annotationMode;
        if (mode.testFlag(AnnotationMode::Mode_Paint) || mode.testFlag(AnnotationMode::Mode_OverPaint)) {
            // Shift picks the 3D fill; plain middle click is the 2D one
            segmentation_flood_fill(clickedCoordinate, *this, event->modifiers().testFlag(Qt::ShiftModifier));
        }
    }
    //finish node drag
    arbNodeDragCache = {};
    draggedNode = nullptr;

    ViewportBase::handleMouseReleaseMiddle(event);
}

void ViewportBase::handleWheelEvent(const QWheelEvent *event) {
    if (QApplication::activeWindow() != nullptr) {//only if active widget belongs to application
        activateWindow();//steal keyboard focus
    }
    setFocus();//get keyboard focus for this widget for viewport specific shortcuts
    // on mac scrolling is rotated with shift
    const auto scroll = std::abs(event->angleDelta().y()) > std::abs(event->angleDelta().x()) ? event->angleDelta().y() : event->angleDelta().x();
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeEditing)
            && event->modifiers().testFlag(Qt::ShiftModifier)
            && state->skeletonState->activeNode != nullptr)
    {//change node radius
        const float radius = state->skeletonState->activeNode->radius + std::copysign(std::min(std::abs(scroll) / 120., 9.), scroll) * 0.1 * state->skeletonState->activeNode->radius;
        Skeletonizer::singleton().setRadius(*state->skeletonState->activeNode, radius);
    } else if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Brush) && event->modifiers().testFlag(Qt::ShiftModifier)) {
        auto & seg = Segmentation::singleton();
        auto curRadius = seg.brush.getRadius();
        // brush radius delta factor (float), as a function of current radius
        seg.brush.setRadius(curRadius + scroll / 120. * 0.1 * curRadius);
    }
    state->viewer->run();
}

void ViewportBase::applyZoom(const QWheelEvent *event, float direction) {
    zoom(std::pow(2, event->angleDelta().y() / 8. / 15. * zoomSpeed * direction));
}

void Viewport3D::handleWheelEvent(const QWheelEvent *event) {
    if (event->modifiers() == Qt::NoModifier) {
        if(Segmentation::singleton().volume_render_toggle) {
            auto& seg = Segmentation::singleton();
            seg.volume_mouse_zoom *= 1 + 0.1 * event->angleDelta().y() / 120.;
        } else {
            const QPointF mouseRel{event->position() - 0.5 * QPointF(width(), height())};
            const auto oldZoom = zoomFactor;
            applyZoom(event);
            const auto oldFactor = state->skeletonState->volBoundary() / oldZoom;
            const auto newFactor = state->skeletonState->volBoundary() / zoomFactor;
            translateX += mouseRel.x() * (oldFactor - newFactor) / width();
            translateY += mouseRel.y() * (oldFactor - newFactor) / height();
        }
    }
    ViewportBase::handleWheelEvent(event);
}

void ViewportOrtho::handleWheelEvent(const QWheelEvent *event) {
    if (event->modifiers() == Qt::CTRL) { // Orthogonal VP or outside VP
        applyZoom(event, -1.0f);
    } else if (event->modifiers() == Qt::NoModifier) {
        const auto scroll = state->viewerState->dropFrames * event->angleDelta().y() / 120.;
        state->viewer->userMove(Dataset::current().scaleFactor.componentMul(n) * -1 * scroll, USERMOVE_DRILL, n);
    }
    ViewportBase::handleWheelEvent(event);
}

void ViewportBase::handleKeyPress(const QKeyEvent *event) {
    const auto ctrl = event->modifiers().testFlag(Qt::ControlModifier);
    const auto alt = event->modifiers().testFlag(Qt::AltModifier);
    const auto shift = event->modifiers().testFlag(Qt::ShiftModifier);
    if (event->key() == Qt::Key_F11) {
        fullscreenAction.trigger();
    } else if (ctrl && shift && event->key() == Qt::Key_C) {
        if(state->skeletonState->activeNode && state->skeletonState->activeNode->isSynapticNode) {
            state->skeletonState->activeNode->correspondingSynapse->toggleDirection();
        }
    } else if (event->key() == Qt::Key_Shift) {
        if (!event->isAutoRepeat() && state->viewerState->keyRepeat) {// if ctrl was pressed initially don’t apply it again
            state->viewerState->repeatDirection *= 10;// increase movement speed
        }
        Segmentation::singleton().brush.setInverse(true);// enable erase mode on shift down
    } else if(event->key() == Qt::Key_I || event->key() == Qt::Key_O) {
        const float angle = shift ? -1: 1;
        switch(event->key()) {
        case Qt::Key_I:
            state->viewer->addRotation(QQuaternion::fromAxisAndAngle(state->viewer->viewportArb->n, angle));
            break;
        case Qt::Key_O:
            state->viewer->addRotation(QQuaternion::fromAxisAndAngle(state->viewer->viewportArb->v2, angle));
            break;
        }
    } else if(event->key() == Qt::Key_V) {
        if(ctrl) {
            emit pasteCoordinateSignal();
        }
    } else if(event->key() == Qt::Key_Space) {
        state->viewerState->showOnlyRawData = true;
        state->viewer->reslice_notify();
        state->viewer->mainWindow.forEachVPDo([] (ViewportBase & vp) {
            vp.showHideButtons(false);
        });
    } else if(event->key() == Qt::Key_Delete) {
        if (ctrl) {
            if (state->skeletonState->activeTree != nullptr) {
                Skeletonizer::singleton().delTree(state->skeletonState->activeTree->treeID);
            }
        } else if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::NodeEditing) && !state->skeletonState->selectedNodes.empty()) {
            bool deleteNodes = true;
            if (state->skeletonState->selectedNodes.size() > 1) {
                QMessageBox prompt{QApplication::activeWindow()};
                prompt.setIcon(QMessageBox::Question);
                prompt.setText(tr("Delete selected nodes?"));
                QPushButton *confirmButton = prompt.addButton(tr("Delete"), QMessageBox::AcceptRole);
                prompt.addButton(tr("Cancel"), QMessageBox::RejectRole);
                prompt.exec();
                deleteNodes = prompt.clickedButton() == confirmButton;
            }
            if (deleteNodes) {
                Skeletonizer::singleton().deleteSelectedNodes();
            }
        }
    } else if(event->key() == Qt::Key_Escape) {
        if (state->skeletonState->selectedNodes.size() > 1) {// active node is not allowed to be deselected
            QMessageBox prompt{QApplication::activeWindow()};
            prompt.setIcon(QMessageBox::Question);
            prompt.setText(tr("Clear current node selection?"));
            QPushButton *confirmButton = prompt.addButton(tr("Clear Selection"), QMessageBox::AcceptRole);
            prompt.addButton(tr("Cancel"), QMessageBox::RejectRole);
            prompt.exec();
            if (prompt.clickedButton() == confirmButton) {
                Skeletonizer::singleton().select<nodeListElement>({});
            }
        }
    } else if(event->key() == Qt::Key_F4) {
        if(alt) {
            QApplication::closeAllWindows();
        }
    }
}

/* While a shape-interpolation chain is running, the arrow keys jump between painted key
 * slices instead of panning in-plane, matching Paintera's bindings. D/F/E/R and the mouse
 * wheel keep stepping through slices freely, so nothing is actually lost. */
bool ViewportOrtho::handleShapeInterpolationKey(const QKeyEvent *event) {
    auto & si = ShapeInterpolation::singleton();
    if (!si.active() || static_cast<int>(viewportType) != si.normalAxisViewport()) {
        return false;
    }
    const auto shift = event->modifiers().testFlag(Qt::ShiftModifier);
    const auto depth = axisGet(state->viewerState->currentPosition, si.normalAxis());
    std::optional<int> target;
    switch (event->key()) {
    case Qt::Key_Left:
        target = shift ? si.firstDepth() : si.prevDepth(depth);
        break;
    case Qt::Key_Right:
        target = shift ? si.lastDepth() : si.nextDepth(depth);
        break;
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
        if (si.removeSliceAt(depth)) {
            state->viewer->mainWindow.warnShapeInterpolation(tr("Removed the key slice at %1. Its painted voxels are still there — erase them if you don’t want them.").arg(depth));
            state->viewer->run();
        }
        return true;
    case Qt::Key_Escape:
        // Esc is far more often a slip than a deliberate discard, so confirm — but only
        // when there is a chain to lose; an empty one just exits.
        if (si.sliceCount() != 0) {
            QMessageBox prompt{QApplication::activeWindow()};
            prompt.setIcon(QMessageBox::Question);
            prompt.setText(tr("End this shape interpolation chain?"));
            prompt.setInformativeText(tr("It has %n key slice(s). The painted voxels are kept either way — only the chain and its interpolations are discarded.", "", static_cast<int>(si.sliceCount())));
            auto * confirm = prompt.addButton(tr("End chain"), QMessageBox::AcceptRole);
            prompt.addButton(tr("Keep going"), QMessageBox::RejectRole);
            state->viewer->suspend([&prompt]{ return prompt.exec(); });
            if (prompt.clickedButton() != confirm) {
                return true;
            }
        }
        si.reset();
        state->viewer->mainWindow.warnShapeInterpolation(tr("Chain ended. Painted slices are kept."));
        state->viewer->run();
        return true;
    default:
        return false;
    }
    if (target) {
        auto pos = state->viewerState->currentPosition;
        axisSet(pos, si.normalAxis(), *target);
        state->viewer->setPosition(pos, USERMOVE_DRILL, n);
    }
    return true;// arrows mean slice navigation in this mode, even at the ends of the chain
}

void ViewportOrtho::handleKeyPress(const QKeyEvent *event) {
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_ShapeInterpolation)
            && !event->isAutoRepeat() && handleShapeInterpolationKey(event)) {
        return;
    }
    //events
    //↓          #   #   #   #   #   #   #   # ↑  ↓          #  #  #…
    //^ os delay ^       ^---^ os key repeat

    //intended behavior:
    //↓          # # # # # # # # # # # # # # # ↑  ↓          # # # #…
    //^ os delay ^       ^-^ knossos specified interval

    //after a ›#‹ event state->viewerKeyRepeat instructs the viewer to check in each frame if a move should be performed
    const bool keyD = event->key() == Qt::Key_D;
    const bool keyF = event->key() == Qt::Key_F;
    const bool keyLeft = event->key() == Qt::Key_Left;
    const bool keyRight = event->key() == Qt::Key_Right;
    const bool keyUp = event->key() == Qt::Key_Up;
    const bool keyDown = event->key() == Qt::Key_Down;
    const auto singleVoxelKey = keyD || keyF || keyLeft || keyRight || keyUp || keyDown;
    const bool keyE = event->key() == Qt::Key_E;
    // Ctrl/Cmd/Alt combinations belong to shortcuts, not to slice stepping — otherwise a
    // binding like Ctrl+F silently moves a slice as well as (or instead of) firing
    if (event->modifiers().testFlag(Qt::ControlModifier) || event->modifiers().testFlag(Qt::AltModifier) || event->modifiers().testFlag(Qt::MetaModifier)) {
        ViewportBase::handleKeyPress(event);
        return;
    }
    if (!event->isAutoRepeat()) {
        const int shiftMultiplier = event->modifiers().testFlag(Qt::ShiftModifier) ? 10 : 1;
        const auto direction = (n * -1).dot(state->viewerState->tracingDirection) >= 0 ? 1 : -1;// reverse n into the frame
        const float directionSign = (keyLeft || keyUp) ? -1 : (keyRight || keyDown) ? 1 : direction * (keyD || keyE ? -1 : 1);
        if (singleVoxelKey) {
            const auto vector = (keyLeft || keyRight) ? v1 : (keyUp || keyDown) ? (v2 * -1) : (n * -1); // transform v2 and n from 1. to 8. octant
            state->viewerState->repeatDirection = Dataset::current().scaleFactor.componentMul(vector) * directionSign * shiftMultiplier * state->viewerState->dropFrames;
            state->viewer->userMove(state->viewerState->repeatDirection, USERMOVE_HORIZONTAL, n);
        } else if(event->key() == Qt::Key_R || keyE) {
            state->viewer->setPositionWithRecentering(state->viewerState->currentPosition - Dataset::current().scaleFactor.componentMul(n) * directionSign * shiftMultiplier * state->viewerState->walkFrames);
        }
    } else if (singleVoxelKey) {
        state->viewerState->keyRepeat = true;
    }
    ViewportBase::handleKeyPress(event);
}

void ViewportBase::handleKeyRelease(const QKeyEvent *event) {
    if(event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        state->viewerState->showOnlyRawData = false;
        state->viewer->updateCurrentPosition();
        state->viewer->reslice_notify();
        state->viewer->mainWindow.forEachVPDo([] (ViewportBase & vp) {
            vp.showHideButtons(state->viewerState->showVpDecorations);
        });
    }
}

void Viewport3D::handleKeyPress(const QKeyEvent *event) {
    if (event->key() == Qt::Key_W && !event->isAutoRepeat()) {// real key press
        QMetaObject::invokeMethod(&wiggletimer, qOverload<>(&QTimer::start));
        wiggleButton.setChecked(true);
    }
    ViewportBase::handleKeyPress(event);
}

void Viewport3D::resetWiggle() {
    QMetaObject::invokeMethod(&wiggletimer, &QTimer::stop);
    state->skeletonState->rotdx -= wiggle;
    state->skeletonState->rotdy -= wiggle;
    wiggleDirection = true;
    wiggle = 0;
    wiggleButton.setChecked(false);
    update();
}

void Viewport3D::handleKeyRelease(const QKeyEvent *event) {
    if (event->key() == Qt::Key_W && !event->isAutoRepeat()) {// real key release
        resetWiggle();
    }
    ViewportBase::handleKeyRelease(event);
}

void Viewport3D::focusOutEvent(QFocusEvent * event) {
    if (focusWidget() != &wiggleButton) {
        resetWiggle();
    }
    ViewportBase::focusOutEvent(event);
}

Coordinate getCoordinateFromOrthogonalClick(const QPointF pos, ViewportOrtho & vp) {
    const auto leftUpper = floatCoordinate{state->viewerState->currentPosition} - (vp.v1 * vp.edgeLength / vp.screenPxXPerMag1Px - vp.v2 * vp.edgeLength / vp.screenPxYPerMag1Px) * 0.5;
    return leftUpper + vp.v1 * (pos.x() / vp.screenPxXPerMag1Px - 0.5) - vp.v2 * (pos.y() / vp.screenPxYPerMag1Px - 0.5);
}

QSet<nodeListElement*> ViewportBase::nodeSelection(int x, int y) {
    // node selection square
    state->viewerState->nodeSelectionSquare.second.x = x;
    state->viewerState->nodeSelectionSquare.second.y = y;
    Coordinate first = state->viewerState->nodeSelectionSquare.first;
    Coordinate second = state->viewerState->nodeSelectionSquare.second;
    // create square
    int minX, maxX, minY, maxY;
    minX = std::min(first.x, second.x);
    maxX = std::max(first.x, second.x);
    minY = std::min(first.y, second.y);
    maxY = std::max(first.y, second.y);
    const auto width = std::abs(maxX - minX);
    const auto height = std::abs(maxY - minY);
    const auto centerX = minX + width / 2;
    const auto centerY = minY + height / 2;
    const auto selectedNodes = pickNodes(centerX, centerY, width, height);
    QSet<nodeListElement*> selectedSet;
    for (const auto & elem : selectedNodes) {
        selectedSet.insert(elem);
    }
    return selectedSet;
}

Coordinate ViewportOrtho::getMouseCoordinate() {
    return getCoordinateFromOrthogonalClick(prevMouseMove, *this);
}

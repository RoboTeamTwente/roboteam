//
// Created by Dawid Kulikowski on 28/12/2021.
//

#include "InterfaceFieldView.h"

InterfaceFieldView::InterfaceFieldView(std::weak_ptr<InterfaceFieldStateStore> state, QWidget* parent):
      QGraphicsView(parent),
      renderer(state),
      state(state) {
    this->setCacheMode(CacheModeFlag::CacheBackground);

    //    this->setViewportUpdateMode(ViewportUpdateMode::FullViewportUpdate);
    this->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    this->setDragMode(QGraphicsView::NoDrag);
    this->setOptimizationFlag(QGraphicsView::DontSavePainterState);

    this->resetCachedContent();
}

void InterfaceFieldView::drawBackground(QPainter *painter, const QRectF &rect) {
    if (auto availableState = this->state.lock()) {
        auto currentState = availableState->getState();
        if (!currentState.has_value()) {
            painter->drawText(0, 0, "Waiting for world...");
        } else {
            const auto field = currentState->field().field();
            this->renderer.updateScale(rect.width(), rect.height(), field.field_length(), field.field_width());

            this->renderer.renderField(painter, *currentState, rect.toRect());
        }
    }
}

void InterfaceFieldView::resizeEvent(QResizeEvent* event) {
    if (scene()) {
        scene()->setSceneRect(viewport()->rect());
    }


    QGraphicsView::resizeEvent(event);
}

#include <InterfaceRobotPathsRenderer.h>
#include <QGraphicsView>
#include <cmath>

namespace rtt::Interface {
    InterfaceRobotPathItem::InterfaceRobotPathItem(rtt::Team team, QGraphicsItem *parent)
        : QGraphicsItem(parent)
    {
        this->team = team;
        this->scale = 0.0;
    }

    void InterfaceRobotPathItem::triggerUpdate(const std::vector<rtt::RobotPath>& paths) {
        if (this->scene()->views().empty()) {
            return;
        }

        if (!this->isVisible()) {
            this->setVisible(true);
        }

        this->paths = paths;
        this->setPos(this->scene()->views()[0]->viewport()->width()/2, this->scene()->views()[0]->viewport()->height()/2);
        this->update();

    }

    void InterfaceRobotPathItem::updateScale(double fieldWidth, double fieldHeight) {
        if (fieldWidth == 0 || fieldHeight == 0) this->scale = 0.0;

        double canvasWidth = static_cast<double>(this->scene()->views()[0]->viewport()->width());
        double canvasHeight = static_cast<double>(this->scene()->views()[0]->viewport()->height());

        double widthScale = canvasWidth / (fieldWidth/1000);
        double heightScale = canvasHeight / (fieldHeight/1000);

        this->scale = std::fmin(widthScale, heightScale);
    }

    void InterfaceRobotPathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
        painter->save();

        painter->setPen(QPen(Qt::cyan, 3, Qt::SolidLine, Qt::RoundCap));

        for (const auto& path : this->paths) {
            auto beginIt = path.points.begin();
            auto nextIt = std::next(beginIt);

            while (beginIt != path.points.end() && nextIt != path.points.end()) {
                QPointF begin(this->scale * beginIt->x, this->scale * beginIt->y);
                QPointF next(this->scale * nextIt->x, this->scale * nextIt->y);

                painter->drawLine(begin.x(), begin.y(), next.x(), next.y());

                beginIt = nextIt;
                std::advance(nextIt, 1);
            } std::cout << std::endl;
        }
        painter->restore();
    }

    QRectF InterfaceRobotPathItem::boundingRect() const {
        return QRectF(-this->scene()->views()[0]->viewport()->width(), -this->scene()->views()[0]->viewport()->height(), this->scene()->views()[0]->viewport()->width()*2, this->scene()->views()[0]->viewport()->height()*2);
    }

}


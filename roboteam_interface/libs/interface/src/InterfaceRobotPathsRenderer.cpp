#include <InterfaceRobotPathsRenderer.h>
#include <QGraphicsView>
#include <cmath>
#include <vector>
#include <ranges>
#include <numeric>

namespace rtt::Interface {
    InterfaceRobotPathItem::InterfaceRobotPathItem(rtt::Team team, int id, QGraphicsItem *parent)
        : QGraphicsPathItem(parent)
    {
        this->team = team;
        this->robot_id = id;
        this->scale = 0.0;
    }

    void InterfaceRobotPathItem::triggerUpdate(const proto::RobotPath& path) {
        if (this->scene()->views().empty()) {
            return;
        }

        this->setPos(this->scene()->views()[0]->viewport()->width()/2, this->scene()->views()[0]->viewport()->height()/2);

        QPainterPath currentPath;
        currentPath.translate({static_cast<qreal>(this->scene()->views()[0]->viewport()->width()/2), static_cast<qreal>(this->scene()->views()[0]->viewport()->height()/2)});
//        currentPath.moveTo();
        for (const auto& point : path.points() ) {
            // Currently std::views:: is broken on my clang++, fix this later
            // TODO: Use c++20 views
            if (currentPath.currentPosition().toPoint() == QPoint{0,0}) {
                currentPath.moveTo({this->scale * point.x(), this->scale * point.y()});
            } else {
                currentPath.lineTo({this->scale * point.x(), this->scale * point.y()});
            }

        }

        this->setPath(currentPath);
    }

    void InterfaceRobotPathItem::updateScale(double fieldWidth, double fieldHeight) {
        if (fieldWidth == 0 || fieldHeight == 0) this->scale = 0.0;

        double canvasWidth = static_cast<double>(this->scene()->views()[0]->viewport()->width());
        double canvasHeight = static_cast<double>(this->scene()->views()[0]->viewport()->height());

        double widthScale = canvasWidth / (fieldWidth/1000);
        double heightScale = canvasHeight / (fieldHeight/1000);

        this->scale = std::fmin(widthScale, heightScale);
    }
    rtt::Team InterfaceRobotPathItem::getTeam() {
        return this->team;
    }

    int InterfaceRobotPathItem::getRobotId() {
        return this->robot_id;
    }

    //    void InterfaceRobotPathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
////        painter->save();
////
////        painter->setPen(QPen(Qt::cyan, 3, Qt::SolidLine, Qt::RoundCap));
////
////        for (const auto& path : this->paths) {
////            auto beginIt = path.points.begin();
////            auto nextIt = std::next(beginIt);
////
////            while (beginIt != path.points.end() && nextIt != path.points.end()) {
////                QPointF begin(this->scale * beginIt->x, this->scale * beginIt->y);
////                QPointF next(this->scale * nextIt->x, this->scale * nextIt->y);
////
////                painter->drawLine(begin.x(), begin.y(), next.x(), next.y());
////
////                beginIt = nextIt;
////                std::advance(nextIt, 1);
////            } std::cout << std::endl;
////        }
////        painter->restore();
//    }


}


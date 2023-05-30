//
// Created by mrlukasbos on 22-9-19.
//
#include <QtCharts/QtCharts>

#ifndef RTT_GRAPHWIDGET_H
#define RTT_GRAPHWIDGET_H

namespace rtt::ai::interface {
/**
 * @brief Class that defines the GraphWidget.
 * The GraphWidget is responsible for showing graphs in the interface. It inherits from QWidget.
 */
class GraphWidget : public QWidget {
    Q_OBJECT
   private:
    float seriesIndex = 0; /**< Time (s) since start */
    float fpsGraphYMax = 0; /**< Maximum value shown on the Y axis */
    float fpsGraphXMin = 0; /**< Minimum value shown on the x axis */
    float fpsGraphXMax = 0; /**< Maximum value shown on the x axis */
    QChartView *fpsView; /**< FPS graph */
    QLineSeries *fpsSeries; /**< FPS values */

   public:
    /**
     * @brief Explicit constructor for the GraphWidget class.
     * @param parent The QWidget this widget is a part of.
     */
    explicit GraphWidget(QWidget *parent = nullptr);
   public slots:
    /**
     * @brief Updates the contents of the graph
     */
    void updateContents();
};

}  // namespace rtt::ai::interface

#endif  // RTT_GRAPHWIDGET_H

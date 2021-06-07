//
// Created by alexander on 9/17/20.
//

#ifndef RVIZ_QUICK_RVIZ_OPTIONS_H
#define RVIZ_QUICK_RVIZ_OPTIONS_H

#include "quick_rviz_object.h"
#include <tf2_ros/transform_listener.h>

namespace rviz
{

class QuickRvizOptions: public QuickRvizObject
{
  Q_OBJECT
  Q_PROPERTY(QString fixedFrame READ getFixedFrame WRITE setFixedFrame NOTIFY fixedFrameChanged)
  Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor NOTIFY backgroundColorChanged)
  Q_PROPERTY(int frameRate READ getFrameRate WRITE setFrameRate NOTIFY frameRateChanged)
  Q_PROPERTY(bool defaultLight READ getDefaultLight WRITE setDefaultLight NOTIFY defaultLightChanged)
  Q_PROPERTY(QStringList frameList READ getFrameList WRITE setFrameList NOTIFY frameListChanged)

public:
  explicit QuickRvizOptions(QObject* parent = Q_NULLPTR);
  ~QuickRvizOptions() override;

  const QString getFixedFrame();
  const QColor getBackgroundColor();
  int getFrameRate();
  bool getDefaultLight();
  const QStringList getFrameList();

public Q_SLOTS:
  void setFixedFrame(const QString &frame);
  void setBackgroundColor(const QColor &color);
  void setFrameRate(int fps);
  void setDefaultLight(bool value);
  void updateFrameList();
  void setFrameList(const QStringList& list);

Q_SIGNALS:
  void fixedFrameChanged(const QString &frame);
  void backgroundColorChanged(const QColor &color);
  void frameRateChanged(int fps);
  void defaultLightChanged(bool value);
  void frameListChanged(const QStringList& list);

private Q_SLOTS:
  void initialize() override;
  void updateProperties();

private:
  bool initialized_;
  QString fixedFrame_;
  QColor backgroundColor_;
  int frameRate_;
  bool defaultLight_;
  tf2_ros::Buffer* tfBuffer_;
  tf2_ros::TransformListener* tfListener_;
  std::vector<std::string> frames_list_;
  QStringList frames_Qlist_;
};

}  // namespace rviz
#endif // RVIZ_QUICK_RVIZ_OPTIONS_H

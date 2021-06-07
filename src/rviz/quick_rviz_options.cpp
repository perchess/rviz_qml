//
// Created by alexander on 9/17/20.
//
#include "quick_rviz_options.h"

#include "visualization_manager.h"
#include "display_group.h"


QStringList vectorStringtoQStringList(const std::vector<std::string>& in)
{
  QStringList out;
  for (auto& it: in)
  {
    out << QString::fromStdString(it);
  }
  return out;
}


namespace rviz
{

QuickRvizOptions::QuickRvizOptions(QObject *parent)
  : QuickRvizObject(parent)
  , initialized_(false)
  , backgroundColor_(QColor("black"))
  , frameRate_(30)
  , defaultLight_(true)
  , tfBuffer_(new tf2_ros::Buffer())
  , tfListener_(new tf2_ros::TransformListener(*tfBuffer_))
{

}

QuickRvizOptions::~QuickRvizOptions()
{
}

const QString QuickRvizOptions::getFixedFrame()
{
  return fixedFrame_;
}

const QColor QuickRvizOptions::getBackgroundColor()
{
  return backgroundColor_;
}

const QStringList QuickRvizOptions::getFrameList()
{
  frames_Qlist_ = vectorStringtoQStringList(frames_list_);
  return frames_Qlist_;
}

int QuickRvizOptions::getFrameRate()
{
  return frameRate_;
}

bool QuickRvizOptions::getDefaultLight()
{
  return frameRate_;
}

void QuickRvizOptions::setFixedFrame(const QString &frame)
{
  if (frame == fixedFrame_) {
    return;
  }
  fixedFrame_ = frame;
  Q_EMIT fixedFrameChanged(frame);
  updateProperties();
}

void QuickRvizOptions::setBackgroundColor(const QColor& color)
{
  if (color == backgroundColor_) {
    return;
  }
  backgroundColor_ = color;
  Q_EMIT backgroundColorChanged(color);
}

void QuickRvizOptions::setFrameList(const QStringList &list)
{
  if (list == frames_Qlist_)
    return;

  frames_Qlist_ = list;
  Q_EMIT frameListChanged(frames_Qlist_);
}


void QuickRvizOptions::setFrameRate(int fps)
{
  if (fps == frameRate_) {
    return;
  }
  frameRate_ = fps;
  Q_EMIT frameRateChanged(fps);
}

void QuickRvizOptions::setDefaultLight(bool value)
{
  if (value == defaultLight_) {
    return;
  }
  defaultLight_ = value;
  Q_EMIT defaultLightChanged(value);
}

void QuickRvizOptions::initialize()
{
  initialized_ = true;
  updateProperties();
}

void QuickRvizOptions::updateFrameList()
{
  tfBuffer_->_getFrameStrings(frames_list_);
}


void QuickRvizOptions::updateProperties()
{
  if (!initialized_ or !getFrame()) {
    return;
  }

  auto dpGroup = getFrame()->getManager()->getRootDisplayGroup()->subProp("Global Options");
  dpGroup->subProp("Fixed Frame")->setValue(fixedFrame_);
  dpGroup->subProp("Background Color")->setValue(backgroundColor_.name());
  dpGroup->subProp("Frame Rate")->setValue(frameRate_);
  dpGroup->subProp("Default Light")->setValue(defaultLight_);
}

}  // namespace rviz

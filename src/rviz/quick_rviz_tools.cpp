//
// Created by alexander on 9/17/20.
//
#include "quick_rviz_tools.h"

#include "visualization_manager.h"
#include "tool_manager.h"

namespace rviz
{

QuickRvizTools::QuickRvizTools(QObject* parent)
  : QuickRvizObject(parent)
  , initialized_(false)
{
  connect(this, &QuickRvizTools::toolNamesChanged, this, &QuickRvizTools::initTools);
}

QuickRvizTools::~QuickRvizTools()
{
  removeTools();
}

const QVariantMap QuickRvizTools::getToolNames()
{
    return toolNames_;
}

const QString QuickRvizTools::getDefaultTool()
{
  return defaultToolName_;
}

void QuickRvizTools::setToolNames(const QVariantMap& toolNames)
{
  if (toolNames_ == toolNames) {
    return;
  }
  toolNames_ = toolNames;
  Q_EMIT toolNamesChanged();
}

bool QuickRvizTools::setCurrentTool(const QString& name)
{
  if (tools_.count(name) == 0) {
    return false;
  }
  auto tool = tools_[name];
  const auto toolManager = getFrame()->getManager()->getToolManager();
  toolManager->setCurrentTool(tool);
  return true;
}

void QuickRvizTools::setPropertyValue(const QString& toolName, const QString& key, const QVariant& value)
{
  if (!initialized_)
    return;

  auto keys = key.split('/');
  Property* prop = tools_[toolName]->getPropertyContainer();
  for (const auto &k: keys)
    prop = prop->subProp(k);

  prop->setValue(value);
}

void QuickRvizTools::setDefaultTool(const QString &name)
{
    if (defaultToolName_ == name)
        return;
    defaultToolName_ = name;

    const auto frame = getFrame();
    ToolManager* toolManager;
    if (frame)
      toolManager = frame->getManager()->getToolManager();
    if (!toolManager)
        return;

    toolManager->setDefaultTool(tools_[defaultToolName_]);
    Q_EMIT defaultToolChanged();
}

void QuickRvizTools::initialize()
{
  initialized_ = true;

  if (tools_.empty()) {
    initTools();
  }
}

void QuickRvizTools::removeTools()
{
  const auto frame = getFrame();
  if (frame)
  {
    const auto toolManager = frame->getManager()->getToolManager();
    toolManager->removeAll();
  }
  tools_.clear();
}

void QuickRvizTools::initTools()
{
  if (!initialized_) {
    return;
  }

  removeTools();

  const auto toolManager = getFrame()->getManager()->getToolManager();
  for (const auto &mapNode: toolNames_.toStdMap())
  {
    auto tool = toolManager->addTool(mapNode.second.toString());
    if (tool) {
      tools_[mapNode.first] = tool;
    }
  }
  if (!tools_.empty()) {
    Q_EMIT toolsCreated();
  }
}

}  // namespace rviz

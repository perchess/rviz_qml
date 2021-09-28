//
// Created by alexander on 9/17/20.
//

#ifndef RVIZ_QUICK_RVIZ_TOOLS_H
#define RVIZ_QUICK_RVIZ_TOOLS_H

#include "quick_rviz_object.h"
#include "properties/property.h"

#include "tool.h"

namespace rviz {

class QuickRvizTools: public QuickRvizObject
{
  Q_OBJECT
  Q_PROPERTY(QVariantMap toolNames READ getToolNames WRITE setToolNames NOTIFY toolNamesChanged)
  Q_PROPERTY(QString defaultToolName READ getDefaultTool WRITE setDefaultTool NOTIFY defaultToolChanged)

public:
  explicit QuickRvizTools(QObject* parent = Q_NULLPTR);
  ~QuickRvizTools() override;

  const QVariantMap getToolNames();
  const QString getDefaultTool();

public Q_SLOTS:
  void setToolNames(const QVariantMap &toolNames);
  bool setCurrentTool(const QString &name);
  void setPropertyValue(const QString& toolName, const QString& key, const QVariant& value);
  void setDefaultTool(const QString &name);

Q_SIGNALS:
  void toolNamesChanged();
  void defaultToolChanged();
  void toolsCreated();

private Q_SLOTS:
  void initialize() override;
  void initTools();

private:
  bool initialized_;
  std::map<QString, Tool*> tools_;
//  QStringList toolNames_;
  QVariantMap toolNames_;
  QString defaultToolName_;

  void removeTools();
};

}  // namespace rviz

#endif // RVIZ_QUICK_RVIZ_TOOLS_H

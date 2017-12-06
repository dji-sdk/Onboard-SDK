/*! @file sdk_widgets.hpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_vehicle.hpp"
#include <QComboBox>
#include <QItemDelegate>
#include <QScrollBar>
#include <QTextBrowser>

//! @note widget for GUI
class TurnModeDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  TurnModeDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget*                    parent,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    QComboBox* editor = new QComboBox(parent);
    editor->addItem("Clockwise");
    editor->addItem("Counter-clockwise");
    return editor;
  }
  void setEditorData(QWidget* editor, const QModelIndex& index) const
  {
    QString    text     = index.model()->data(index, Qt::EditRole).toString();
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    int        tindex   = comboBox->findText(text);
    comboBox->setCurrentIndex(tindex);
  }
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const
  {
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    QString    text     = comboBox->currentText();
    model->setData(index, text, Qt::EditRole);
  }
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index __UNUSED) const
  {
    editor->setGeometry(option.rect);
  }
};

class ActionDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  ActionDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget*                    parent,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    QComboBox* editor = new QComboBox(parent);
    editor->addItem("Stay");
    editor->addItem("Take picture");
    editor->addItem("Start recording");
    editor->addItem("Stop recording");
    editor->addItem("Yaw");
    editor->addItem("Gimbal pitch");
    return editor;
  }
  void setEditorData(QWidget* editor, const QModelIndex& index) const
  {
    QString    text     = index.model()->data(index, Qt::EditRole).toString();
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    int        tindex   = comboBox->findText(text);
    comboBox->setCurrentIndex(tindex);
  }
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const
  {
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    QString    text     = comboBox->currentText();
    model->setData(index, text, Qt::EditRole);
  }
  void updateEditorGeometry(QWidget*                    editor,
                            const QStyleOptionViewItem& option __UNUSED,
                            const QModelIndex& index __UNUSED) const
  {
    editor->setGeometry(option.rect);
  }
};

class ReadOnlyDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  ReadOnlyDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget* parent             __UNUSED,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    return NULL;
  }
};

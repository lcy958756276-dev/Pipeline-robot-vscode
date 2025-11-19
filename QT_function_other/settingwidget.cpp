/**
 * @file settingwidget.cpp
 * @brief 系统设置界面类，实现片头信息管理、数据库操作、路径选择及导出功能
 *
 * 功能：
 * - 设置文件保存路径
 * - 数据库增删改查
 * - 片头信息设置
 * - 表格导出到 Excel
 */

#include "settingwidget.h"
#include "ui_settingwidget.h"

/**
 * @brief 构造函数，初始化 UI 和数据库，绑定信号槽
 * @param argc ROS 初始化参数个数
 * @param argv ROS 初始化参数列表
 * @param parent 父 QWidget
 */
SettingWidget(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    node(argc, argv, "test"), // 使用参数初始化node
    ui(new Ui::SettingWidget)
{
    ui->setupUi(this);
    addWindow = new Dialog;

    ui->label_3->clear(); // 清空默认显示
    sqlDatabase = new Sql();
    sqlDatabase->initDatabase();
    slotUpdateTableWidget();

    // 绑定按钮点击事件
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onPushButtonClicked()));
    connect(addWindow, &QDialog::accepted, this, &SettingWidget::slotUpdateTableWidget);

    // 设置文件保存目录
    QString defaultPath = QDir::currentPath() + "/storage";
    QDir dir(defaultPath);
    if (!dir.exists()) dir.mkpath(".");
    ui->label_3->setText(defaultPath);
    ui->label_3->setWordWrap(true);

    // 调整 tableWidget 列宽
    for (int column = 0; column < ui->tableWidget->columnCount(); ++column)
        ui->tableWidget->setColumnWidth(column, 95);
}

/**
 * @brief 析构函数
 */
~SettingWidget() {
    delete ui;
}

/**
 * @brief 弹出文件夹选择对话框，更新路径显示
 */
void onPushButtonClicked() {
    QString directory = QFileDialog::getExistingDirectory(
        this, tr("选择文件夹"), "/home",
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (!directory.isEmpty()) {
        ui->label_3->setText(directory);
    }
}

/**
 * @brief 弹出新增片头信息窗口
 */
void on_addButton_clicked() {
    addWindow->initLineEdit();
    addWindow->initCheckBox();
    addWindow->setMode(Dialog::AddMode);
    addWindow->show();
}

/**
 * @brief 更新 tableWidget 显示数据库内容
 *
 * 查询 headers 表，填充表格。
 */
void slotUpdateTableWidget() {
    qDebug() << "slotUpdateTableWidget函数被激活";
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(0);

    QSqlQuery query;
    if (!query.exec("SELECT ID, DetectionLocation, DetectionDate, Task, TaskNumber, "
                    "PipeType, StartWellNumber, EndWellNumber FROM headers")) {
        qDebug() << "Query failed:" << query.lastError().text();
        return;
    }

    while (query.next()) {
        int rowCount = ui->tableWidget->rowCount();
        ui->tableWidget->insertRow(rowCount);

        ui->tableWidget->setItem(rowCount, 0, new QTableWidgetItem(query.value(0).toString())); // ID
        ui->tableWidget->setItem(rowCount, 1, new QTableWidgetItem(query.value(1).toString())); // DetectionLocation
        ui->tableWidget->setItem(rowCount, 4, new QTableWidgetItem(query.value(2).toDate().toString("yyyy-MM-dd"))); // DetectionDate
        ui->tableWidget->setItem(rowCount, 2, new QTableWidgetItem(query.value(3).toString())); // Task
        ui->tableWidget->setItem(rowCount, 3, new QTableWidgetItem(query.value(4).toString())); // TaskNumber
        ui->tableWidget->setItem(rowCount, 7, new QTableWidgetItem(query.value(5).toString())); // PipeType
        ui->tableWidget->setItem(rowCount, 5, new QTableWidgetItem(query.value(6).toString())); // StartWellNumber
        ui->tableWidget->setItem(rowCount, 6, new QTableWidgetItem(query.value(7).toString())); // EndWellNumber
    }

    ui->tableWidget->setColumnHidden(0, true); // 隐藏 ID 列
    ui->tableWidget->update();
}

/**
 * @brief 删除当前选中行及数据库记录
 */
void on_delButton_clicked() {
    int selectedRow = ui->tableWidget->currentRow();
    if (selectedRow == -1) {
        QMessageBox::warning(this, "警告", "你未选中删除的行");
        return;
    }

    int id = ui->tableWidget->item(selectedRow, 0)->text().toInt();
    sqlDatabase->deleteHeaderFromDatabase(id);
    ui->tableWidget->removeRow(selectedRow);
}

/**
 * @brief 修改当前选中行的数据
 */
void on_changeButton_clicked() {
    int currentRow = ui->tableWidget->currentRow();
    if (currentRow == -1) {
        QMessageBox::warning(this, "警告", "您未选中要修改的行！");
        return;
    }

    int id = ui->tableWidget->item(currentRow, 0)->text().toInt();
    addWindow->setMode(Dialog::EditMode);
    addWindow->setRecordId(id);
    addWindow->show();

    QSqlRecord record = sqlDatabase->queryRecordById(id);
    addWindow->setEdit(
        record.value("DetectionLocation").toString(),
        record.value("DetectionDate").toDate(),
        record.value("DetectionUnit").toString(),
        record.value("Task").toString(),
        record.value("TaskNumber").toString(),
        record.value("Inspector").toString(),
        record.value("PipeType").toString(),
        record.value("PipeMaterial").toString(),
        record.value("TotalPipeLength").toString(),
        record.value("PipeDiameter").toString(),
        record.value("WellDepth").toString(),
        record.value("DetectionDirection").toString(),
        record.value("StartWellNumber").toString(),
        record.value("EndWellNumber").toString(),
        record.value("PipeLength").toString()
    );
}

/**
 * @brief 将选中行设置为片头，并通过 ROS 发布
 */
void on_pianTouButton_clicked() {
    int selectedRow = ui->tableWidget->currentRow();
    if (selectedRow == -1) {
        QMessageBox::warning(this, "警告", "你未选中的行");
        return;
    }

    int id = ui->tableWidget->item(selectedRow, 0)->text().toInt();
    QSqlRecord record = sqlDatabase->queryRecordById(id);

    pipeline::HeaderInfo msg;
    msg.id = record.value("ID").toInt();
    msg.location = record.value("DetectionLocation").toString().toStdString();
    msg.date = record.value("DetectionDate").toString().toStdString();
    msg.unit = record.value("DetectionUnit").toString().toStdString();
    msg.task = record.value("Task").toString().toStdString();
    msg.taskNumber = record.value("TaskNumber").toString().toStdString();
    msg.inspector = record.value("Inspector").toString().toStdString();
    msg.pipeType = record.value("PipeType").toString().toStdString();
    msg.pipeMaterial = record.value("PipeMaterial").toString().toStdString();
    msg.totalPipeLength = record.value("TotalPipeLength").toDouble();
    msg.pipeDiameter = record.value("PipeDiameter").toDouble();
    msg.wellDepth = record.value("WellDepth").toDouble();
    msg.detectionDirection = record.value("DetectionDirection").toString().toStdString();
    msg.startWellNumber = record.value("StartWellNumber").toString().toStdString();
    msg.endWellNumber = record.value("EndWellNumber").toString().toStdString();
    msg.pipeLength = record.value("PipeLength").toDouble();

    node.publishPianTou(msg);
    QMessageBox::warning(this, "提示", "设置成功");
}

/**
 * @brief 导出数据库表 headers 到 Excel 文件
 */
void on_pushButton_2_clicked() {
    sqlDatabase->exportTableToExcel("headers");
}

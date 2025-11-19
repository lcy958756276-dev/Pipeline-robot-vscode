/**
 * @file jiantuwidget.cpp
 * @brief JianTuWidget 类的实现文件，包含 UI 初始化、布局设置及 ROS 节点初始化
 */

/**
 * @brief 构造函数，初始化 JianTuWidget 对象
 * @param argc 命令行参数数量，用于 ROS 节点初始化
 * @param argv 命令行参数数组，用于 ROS 节点初始化
 * @param parent 父 QWidget 指针，默认为 nullptr
 *
 * 构造函数主要功能：
 * - 初始化 UI 界面
 * - 创建并设置 VerticalLayout 为主布局
 * - 使用 argc 和 argv 初始化 ROS 节点 node
 */
JianTuWidget(int argc, char **argv, QWidget *parent = nullptr)
    : QWidget(parent),
      node(argc, argv, "node_jiantu"),  // 使用参数初始化 node
      ui(new Ui::JianTuWidget)
{
    ui->setupUi(this);
    VerticalLayout = new QVBoxLayout(this); // 初始化 VerticalLayout
    setLayout(VerticalLayout);              // 设置为主布局
}

/**
 * @brief 析构函数，释放 JianTuWidget 资源
 *
 * 析构函数主要功能：
 * - 删除 UI 对象，释放资源
 */
~JianTuWidget()
{
    delete ui;
}

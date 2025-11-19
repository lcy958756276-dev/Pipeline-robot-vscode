import os
import sys
from unittest.mock import MagicMock
sys.path.insert(0, os.path.abspath('..'))
MOCK_MODULES = ['rospy', 'std_msgs', 'std_msgs.msg', 'subprocess']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = MagicMock()

project = 'Pipeline_robot_vscode'
author = 'lcy'
copyright = ''

extensions = [
    "sphinx_rtd_theme",
    "recommonmark",
    "sphinx_markdown_tables",
    "sphinx.ext.autodoc",       # 已经在用 automodule
    "sphinx.ext.autosummary",   # 新加
    "sphinx.ext.viewcode",  # 显示源代码链接
    'breathe',
]

# 自动生成 autosummary 文档
autosummary_generate = True


templates_path = ['_templates']

# 设置 HTML 主题
html_theme = "sphinx_rtd_theme"

# 支持 .md 和 .rst
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_static_path = ['_static']

autodoc_mock_imports = [
    'rospy',
    'cv2',
    'docx',
    'CvBridge',
    'cv_bridge',
    'pipeline_robot',
    'nav_msgs',
    'geometry_msgs',
    "onnxruntime",
    "sensor_msgs",
    'tf2_ros',
    'tf2_geometry_msgs',
]

# -- Breathe configuration ---------------------------------------------------
breathe_projects = {
    "vscode_cpp": "./_doxygen/xml"
}
breathe_default_project = "vscode_cpp"
breathe_default_members = ('members', 'undoc-members')



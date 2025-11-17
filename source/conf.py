import os
import sys
sys.path.insert(0, os.path.abspath('../'))

project = 'Pipeline_robot_vscode'
author = 'lcy'
copyright = ''

extensions = [
    "sphinx_rtd_theme",
    "recommonmark",
    "sphinx_markdown_tables",
]

templates_path = ['_templates']

# 设置 HTML 主题
html_theme = "sphinx_rtd_theme"

# 支持 .md 和 .rst
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_static_path = ['_static']

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import sys

sys.path.insert(0, "_extensions")

project = 'Renesas RZ G3S Cortex-M33 Zephyr on SMARC Evaluation Board Kit'
copyright = 'Copyright (c) 2024 EPAM Systems'
author = 'EPAM'
release = 'v1.0.2'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.ifconfig',
    'sphinx.ext.todo',
    'zephyr.application',
    'rst2pdf.pdfbuilder',
    'sphinx.ext.intersphinx',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']

# -- Options for todo extension ----------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/todo.html#configuration

todo_include_todos = True

latex_documents = [('index', 'rzg3s_release_notes.tex', project, author, 'manual')]

latex_elements = {
    'papersize': 'a4paper',
    'classoptions': ',openany,oneside',
    'babel' : '\\usepackage[english]{babel}',
    'fvset' : '\\fvset{fontsize=\\scriptsize}',
    'preamble': r'''
    \usepackage[none]{hyphenat}
    '''
}

latexpdf_documents = [('index', 'rzg3s_release_notes.pdf', project, author, 'manual')]

pdf_documents = [('index', u'pdf-name', u'Sample doc Title', u'author name')]

intersphinx_mapping = {'zephyr': ('https://docs.zephyrproject.org/latest/', None)}

def setup(app):
    app.add_config_value('PDF_GEN', '', '1')

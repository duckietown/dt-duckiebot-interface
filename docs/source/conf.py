# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('..'))
sys.path.insert(0, os.path.abspath('../../catkin_ws/src/'))
import led_emitter
print(sys.path)



# -- Project information -----------------------------------------------------

project = 'duckiebot-interface'
copyright = '2019, Duck Quackermann'
author = 'Duck Quackermann'

# The full version, including alpha/beta/rc tags
release = '0.0.1'


# -- General configuration ---------------------------------------------------


# Napoleon (for Google-style docstrings)
extensions = ['sphinx.ext.napoleon']

# Autodocs
extensions += ['sphinx.ext.autodoc']


# Add intersphynx to connect with the base ROS
# extensions += ['sphinx.ext.intersphinx']
# intersphinx_mapping = {'ros':
#   ('http://otamachan.github.io/sphinxros/indigo/', None)}

# Autodocs Configuration
autoclass_content = 'both'
autodoc_mock_imports = ['cv2',
                        'numpy',
                        'rospy',
                        'Adafruit_PWM_Servo_Driver',
                        'camera_driver',
                        'sensor_msgs',
                        'cv_bridge',
                        'picamera',
                        'thread',
                        'yaml',
                        'duckietown_msgs',
                        'wheels_driver',
                        'Adafruit_MotorHAT']
autodoc_typehints = 'signature'
autodoc_default_options = {'show-inheritance': False}
napoleon_google_docstring = True
napoleon_use_param = False
napoleon_use_ivar = True
napoleon_include_init_with_doc = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# Set the index page
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

#!/bin/bash
#
# A convenient script to create documentation
#
set -ue -o pipefail

###############################
# 0. check needed software
###############################
if (! which pandoc ); then
    echo "Please install pandoc first. Try:"
    echo "  sudo apt install pandoc"
    exit 1
fi

###############################
# 1. Source the underlay
###############################
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
set -u                          # re-enable undefined variable check


###############################
# 2. run tinman's "docs" target
###############################
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select tinman \
       --cmake-target "docs"
##echo "open src/tinman/docs/html/index.html"

###############################
# 3. combine all docs
###############################
DOCS_DIR=docs/html
open $DOCS_DIR/index.html || true

